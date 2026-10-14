#!/usr/bin/env python3
"""Loop-closure error against a physically marked start/end point.

Every accuracy number we have so far is scored against leg odometry, which is itself
a drifting dead-reckoning estimate. This script scores against the floor instead: if
the robot starts and ends on the same marked spot, true displacement between those two
moments is zero (to within how precisely it was repositioned), so whatever the estimator
reports between them is real error.

Protocol the recording must follow:
    1. place the robot on the mark, note its heading
    2. start recording, then STAND STILL ~10 s
    3. walk the loop
    4. return to the mark, align heading
    5. STAND STILL ~10 s, then stop recording

The two dwells are found automatically from leg-odometry speed, so no timestamps need
to be written down.

Usage: tools/eval_loop_closure.py results/<run> data/bags/<bag> [--speed 0.05] [--dwell 3]
"""
import sys, os, math, argparse
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dr_payoff import norm, EST_TOPIC, REF_TOPIC

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def load_full(uri, topic):
    """positions + quaternions + body speed, which `load` does not return."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    T, P, Q, S = [], [], [], []
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Odometry)
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        P.append((m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
        o = m.pose.pose.orientation
        Q.append((o.x, o.y, o.z, o.w))
        t = m.twist.twist.linear
        S.append(math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2))
    return T, P, Q, S


def rotmat(q):
    """quaternion (x, y, z, w) -> 3x3 rotation matrix (body -> world)."""
    x, y, z, w = q
    return np.array([[1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
                     [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
                     [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]])


# NOTE: there is deliberately no `yaw(q)` helper any more. The IMU body frame on this
# robot sits at pitch -84..-88 deg, i.e. next to the ZYX gimbal-lock singularity, so the
# ZYX yaw of a single orientation mixes roll into "yaw" and is meaningless. Every yaw
# figure reported before 2026-10-13 came from that and was wrong. Heading change must be
# taken from the WORLD-frame delta rotation between two orientations -- see rel_rot().


def rel_rot(M0, M1):
    """World-frame rotation from orientation M0 to M1, split into heading (about world z)
    and tilt (residual pitch/roll of the delta). Valid at any body mounting, because the
    delta is near-identity apart from heading, far from any singularity."""
    D = M1 @ M0.T
    heading = math.degrees(math.atan2(D[1, 0], D[0, 0]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -D[2, 0]))))
    roll = math.degrees(math.atan2(D[2, 1], D[2, 2]))
    # tilt = angle between where world-up maps before and after: pure heading leaves it 0
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, D[2, 2]))))
    return heading, tilt, pitch, roll


def dwells(T, S, max_speed, min_len):
    """contiguous spans with speed below max_speed lasting at least min_len seconds"""
    out, start = [], None
    for i, s in enumerate(S):
        if s < max_speed:
            if start is None:
                start = i
        else:
            if start is not None and T[i - 1] - T[start] >= min_len:
                out.append((T[start], T[i - 1]))
            start = None
    if start is not None and T[-1] - T[start] >= min_len:
        out.append((T[start], T[-1]))
    return out


def mean_pose(T, P, Q, t0, t1):
    """mean position and chordal-mean orientation (3x3) over [t0, t1]."""
    idx = [i for i in range(len(T)) if t0 <= T[i] <= t1]
    if not idx:
        return None, None
    c = tuple(sum(P[i][k] for i in idx) / len(idx) for k in range(3))
    U, _, Vt = np.linalg.svd(sum(rotmat(Q[i]) for i in idx))
    M = U @ Vt
    if np.linalg.det(M) < 0:
        U[:, -1] *= -1
        M = U @ Vt
    return c, M


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('result'); ap.add_argument('bag')
    ap.add_argument('--speed', type=float, default=0.05, help='m/s, below this is "still"')
    ap.add_argument('--dwell', type=float, default=3.0, help='s, minimum dwell length')
    a = ap.parse_args()

    rT, rP, rQ, rS = load_full(a.bag, REF_TOPIC)
    eT, eP, eQ, _ = load_full(a.result, EST_TOPIC)

    d = dwells(rT, rS, a.speed, a.dwell)
    t0 = rT[0]
    print(f"=== dwells found (speed < {a.speed} m/s for >= {a.dwell} s) ===")
    if not d:
        print("  NONE. Either the robot never stood still, or --speed is too tight.")
        print("  Without a dwell at each end this bag cannot give loop closure.")
        return 1
    for s, e in d:
        print(f"  t={s-t0:7.1f} -> {e-t0:7.1f}  ({e-s:5.1f} s)")

    first, last = d[0], d[-1]
    if first == last:
        print("\n  Only ONE dwell. The robot must stand still at BOTH ends.")
        return 1
    gap = last[0] - first[1]
    print(f"\n  using first and last; {gap:.1f} s of motion between them")

    print(f"\n=== loop closure (true displacement should be ~0) ===")
    print(f"  {'source':<22} {'drift':>9} {'xy':>9} {'z':>9} {'heading':>9} {'tilt':>7}")
    worst = None
    for name, T, P, Q in (('Point-LIO', eT, eP, eQ), ('leg odometry', rT, rP, rQ)):
        p0, M0 = mean_pose(T, P, Q, *first)
        p1, M1 = mean_pose(T, P, Q, *last)
        if p0 is None or p1 is None:
            print(f"  {name:<22} no poses inside a dwell window")
            continue
        dv = tuple(p1[k] - p0[k] for k in range(3))
        head, tilt, _, _ = rel_rot(M0, M1)
        print(f"  {name:<22} {norm(dv):8.3f}m {math.hypot(dv[0],dv[1]):8.3f}m "
              f"{dv[2]:+8.3f}m {head:+8.2f}d {tilt:6.2f}d")
        if name == 'Point-LIO':
            worst = norm(dv)

    # path length, so drift can be read as a percentage
    plen = sum(math.dist(rP[i - 1], rP[i]) for i in range(1, len(rP)))
    print(f"\n  leg-odometry path length between marks: {plen:.1f} m")
    if worst is not None and plen > 0:
        print(f"  Point-LIO closure error as % of path : {100*worst/plen:.2f} %")
    print(f"\n  heading = rotation about world z (CCW +). tilt = how far 'up' moved;"
          f" truth is ~0 on a flat floor.")
    print(f"\n  NOTE: this is only as good as the repositioning. Measure how precisely\n"
          f"  the robot was put back on the mark and treat that as the error floor.")
    return 0


if __name__ == '__main__':
    sys.exit(main())
