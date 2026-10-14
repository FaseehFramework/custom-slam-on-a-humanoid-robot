#!/usr/bin/env python3
"""Does the DR factor inject false vertical velocity through the attitude estimate?

Our DR residual is  z = R * v_leg - v  (R = filter attitude, v_leg = leg-odometry body
velocity), but the update has no attitude Jacobian, so it treats R * v_leg as a fixed
measurement. With a tilt error, horizontal walking velocity leaks into (R * v_leg)_z and
the filter is pulled toward that false vertical velocity.

Signature, over the walking segment (between first and last dwell):
  corr      correlation of the filter's v_z with (R_est * v_leg)_z
  leak_int  integral of (R_est * v_leg)_z -- what the leak alone would do to height
  tilt_dis  tilt disagreement between filter and leg-odometry attitude (heading-free)

DR on with the bug: corr high, |leak_int| ~ 1 m per minute of walking.
DR off: v_z is not driven by this term, so corr should be markedly lower.
A correct fix should bring DR-on's corr down toward the DR-off value.

Usage: tools/dr_vertical_check.py data/bags/<bag> results/<run> [results/<run> ...]
"""
import sys, os, math, bisect
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from eval_loop_closure import rotmat, dwells, load_full
from dr_payoff import EST_TOPIC, REF_TOPIC


def odo(uri, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out = []
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Odometry)
        o, v = m.pose.pose.orientation, m.twist.twist.linear
        out.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                    (o.x, o.y, o.z, o.w), np.array([v.x, v.y, v.z])))
    return out


def main():
    bag, runs = sys.argv[1], sys.argv[2:]
    leg = odo(bag, REF_TOPIC)
    lt = [x[0] for x in leg]
    rT, _, _, rS = load_full(bag, REF_TOPIC)
    d = dwells(rT, rS, 0.05, 3.0)
    if len(d) >= 2:
        ws, we = d[0][1], d[-1][0]
    else:
        ws, we = lt[0], lt[-1]
    print(f"walking segment {ws - lt[0]:.1f}..{we - lt[0]:.1f} s")
    print(f"  {'run':<44} {'corr':>6} {'leak_int':>9} {'tilt_dis med':>13} {'p90':>6}")
    for run in runs:
        est = odo(run, EST_TOPIC)
        pred, vz, tilt = [], [], []
        for t, q, v in est:
            if not (ws <= t <= we):
                continue
            i = bisect.bisect_left(lt, t)
            if i <= 0 or i >= len(lt):
                continue
            Me, Ml = rotmat(q), rotmat(leg[i][1])
            pred.append((Me @ leg[i][2])[2])
            vz.append(v[2])
            ue, ul = Me.T @ np.array([0, 0, 1.0]), Ml.T @ np.array([0, 0, 1.0])
            tilt.append(math.degrees(math.acos(max(-1.0, min(1.0, float(ue @ ul))))))
        pred, vz, tilt = np.array(pred), np.array(vz), np.array(tilt)
        dt = np.median(np.diff([t for t, _, _ in est]))
        corr = np.corrcoef(pred, vz)[0, 1] if len(vz) > 2 else float('nan')
        print(f"  {os.path.basename(run.rstrip('/')):<44} {corr:>+6.3f} {np.sum(pred) * dt:>+8.3f}m"
              f" {np.median(tilt):>12.2f}d {np.percentile(tilt, 90):>5.1f}d")


if __name__ == '__main__':
    main()
