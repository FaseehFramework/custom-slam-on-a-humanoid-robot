#!/usr/bin/env python3
"""One-line score of a localizer run (localizer_offline output) against what is known.

  jumps     horizontal move of the robot's MAP position caused by each correction update
            (median/p90/max) --
            the correction should move slowly (odometry drift is ~0.2 cm/s), so big jumps
            are inconsistent fixes, which a controller would feel.
  self      walk end minus walk start, as forward/left of the start heading (compare with
            the operator's repositioning report).
  anchor    (optional) walk START minus the anchor walk's END, horizontal: for a test walk
            that began exactly where the mapping walk ended, the truth is 0.

Usage: tools/reloc_metrics.py <loc_out_dir> <walk_result_bag> <walk_data_bag>
                              [<anchor_traj> <anchor_data_bag>]
"""
import sys, os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from eval_loop_closure import load_full, dwells, mean_pose
from closure_in_robot_frame import body_forward
from dr_payoff import EST_TOPIC, REF_TOPIC


def poses(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[EST_TOPIC])); o = {}
    while r.has_next():
        _, raw, _ = r.read_next(); m = deserialize_message(raw, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation; x, y, z, w = q.x, q.y, q.z, q.w
        T = np.eye(4)
        T[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
        T[:3, 3] = [p.x, p.y, p.z]; o[m.header.stamp.sec * 10**9 + m.header.stamp.nanosec] = T
    return o


def dwell_pose(traj, bag, which):
    rT, _, _, rS = load_full(bag, REF_TOPIC); d = dwells(rT, rS, 0.05, 3.0)
    eT, eP, eQ, _ = load_full(traj, EST_TOPIC)
    a, b = d[0] if which == 'first' else d[-1]
    p, M = mean_pose(eT, eP, eQ, a, b); return np.array(p), M


def main():
    out, res, bag = sys.argv[1:4]
    traj = os.path.join(out, 'traj')
    odo, loc = poses(res), poses(traj)
    ks = sorted(odo)
    off = [loc[k] @ np.linalg.inv(odo[k]) for k in ks]
    # The jump a controller feels: how far the robot's MAP position moves when a correction
    # is applied, i.e. old vs new correction applied to the same odometry pose. (Comparing
    # the corrections' translations instead is wrong: a 2 deg yaw tweak with the robot 7 m
    # from the map origin moves that translation ~24 cm while the robot moves ~0.)
    j = np.array([math.hypot(*((off[i] @ odo[ks[i]])[:2, 3] - (off[i - 1] @ odo[ks[i]])[:2, 3]))
                  for i in range(1, len(ks))])
    j = j[j > 1e-9]
    att = np.loadtxt(os.path.join(out, 'attempts.txt'))
    f_body, _, _ = body_forward(bag)
    p0, M0 = dwell_pose(traj, bag, 'first'); p1, _ = dwell_pose(traj, bag, 'last')
    fw = M0 @ f_body; fw[2] = 0; fw /= np.linalg.norm(fw); left = np.cross([0, 0, 1.0], fw)
    d = p1 - p0
    line = (f'accepted {int(att[:, 1].sum())}/{len(att)}  jumps(cm) med {100*np.median(j):.1f} p90 {100*np.percentile(j, 90):.1f} '
            f'max {100*j.max():.1f}  | self: fwd {100*(d @ fw):+.1f} left {100*(d @ left):+.1f} cm')
    if len(sys.argv) >= 6:
        pa, _ = dwell_pose(sys.argv[4], sys.argv[5], 'last')
        line += f'  | start-vs-anchor-end {100*math.hypot(*(p0 - pa)[:2]):.1f} cm'
    print(line)


if __name__ == '__main__':
    main()
