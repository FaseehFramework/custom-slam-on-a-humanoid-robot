#!/usr/bin/env python3
"""Per-stamp pose error of one trajectory bag against a reference trajectory bag.

Usage: tools/compare_traj.py <est_bag> <ref_bag> [--ref-shift x,y,z,yaw_deg]

Both bags hold /aft_mapped_to_init. Poses are matched by exact header stamp. --ref-shift
pre-multiplies the reference by a known gravity-aligned transform (used for the
known-shift localizer self-test). Reports position error (horizontal, vertical) and
rotation error (angle of R_ref^T R_est) as median / p95 / max, plus the first and last
second, so a start-up or end-of-walk problem is visible.
"""
import argparse, math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def load(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    out = {}
    while r.has_next():
        tn, raw, _ = r.read_next()
        if tn != '/aft_mapped_to_init':
            continue
        m = deserialize_message(raw, Odometry)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        out[m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec] = (R, np.array([p.x, p.y, p.z]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('est'); ap.add_argument('ref')
    ap.add_argument('--ref-shift', default=None)
    a = ap.parse_args()
    est, ref = load(a.est), load(a.ref)
    Rs, ts = np.eye(3), np.zeros(3)
    if a.ref_shift:
        x, y, z, yaw = (float(v) for v in a.ref_shift.split(','))
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        Rs, ts = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]), np.array([x, y, z])
    keys = sorted(set(est) & set(ref))
    if not keys:
        sys.exit('no common stamps')
    eh, ev, er = [], [], []
    for k in keys:
        Re, te = est[k]
        Rr, tr = ref[k]
        Rr, tr = Rs @ Rr, Rs @ tr + ts
        d = te - tr
        eh.append(math.hypot(d[0], d[1])); ev.append(d[2])
        er.append(math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(Rr.T @ Re) - 1) / 2)))))
    eh, ev, er = np.array(eh), np.array(ev), np.array(er)
    t = (np.array(keys) - keys[0]) * 1e-9
    print(f'{len(keys)} common stamps ({len(est)} est, {len(ref)} ref), {t[-1]:.1f} s')
    print(f'  {"":<12} {"median":>8} {"p95":>8} {"max":>8}   {"first 1 s":>10} {"last 1 s":>10}')
    for name, e, u in (('horizontal', eh, 'm'), ('vertical', np.abs(ev), 'm'), ('rotation', er, 'deg')):
        f, l = e[t <= 1.0].mean(), e[t >= t[-1] - 1.0].mean()
        print(f'  {name:<12} {np.median(e):>7.3f}{u[0]} {np.percentile(e, 95):>7.3f}{u[0]} {e.max():>7.3f}{u[0]}'
              f'   {f:>9.3f}{u[0]} {l:>9.3f}{u[0]}')


if __name__ == '__main__':
    main()
