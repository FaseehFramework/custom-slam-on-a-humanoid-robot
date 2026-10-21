#!/usr/bin/env python3
"""How much did the CHEST turn relative to the FEET between the first and last stand-still?

The IMU (and so every heading our estimators report) is on the chest. The operator aligns
the FEET to the tape. The waist yaw joint and both hip yaw joints sit between the two, so a
difference in those joint angles between the start and end dwells appears as a "heading
error" that the feet never had.

Reads /aima/hal/joint/{waist,leg}/state (aimdk_msgs/JointStateArray, parsed from CDR: the
aimdk type support is aarch64-only) and averages them over the first and last dwells
(found from leg odometry, as in eval_loop_closure.py).

Joint sign conventions are NOT verified. The script prints raw joint deltas; it does not
claim which combination equals chest-minus-feet yaw.

Usage: tools/stance_heading.py data/bags/<bag>
"""
import sys, os, struct, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rosbag2_py
from decode_locomotion_cmd import CDR
from eval_loop_closure import load_full, dwells
from dr_payoff import REF_TOPIC

TOPICS = ['/aima/hal/joint/waist/state', '/aima/hal/joint/leg/state']
JOINTS = ['waist_yaw_joint', 'left_hip_yaw_joint', 'right_hip_yaw_joint',
          'waist_roll_joint', 'waist_pitch_joint']


def parse(raw):
    c = CDR(raw)
    sec, nsec = c.i32(), c.u32(); c.string(); c.u32(); c.i32(); c.u32()
    c.p += 1                                    # DomainErrorState (uint8)
    out = {}
    for _ in range(c.u32()):
        name = c.string(); pos = c.f64(); c.f64(); c.f64()
        c.align(2); c.p += 2                    # error_code (uint16)
        out[name] = pos
    return sec + nsec * 1e-9, out


def main():
    bag = sys.argv[1]
    rT, _, _, rS = load_full(bag, REF_TOPIC)
    d = dwells(rT, rS, 0.05, 3.0)
    if len(d) < 2:
        sys.exit('need two dwells')
    first, last = d[0], d[-1]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=TOPICS))
    acc = {k: {j: [] for j in JOINTS} for k in ('first', 'last')}
    while r.has_next():
        _, raw, _ = r.read_next()
        t, js = parse(raw)
        for k, (a, b) in (('first', first), ('last', last)):
            if a <= t <= b:
                for j in JOINTS:
                    if j in js:
                        acc[k][j].append(js[j])
    t0 = rT[0]
    print(f'first dwell {first[0]-t0:.1f}-{first[1]-t0:.1f} s, last dwell {last[0]-t0:.1f}-{last[1]-t0:.1f} s')
    print(f'  {"joint":<22} {"first":>8} {"last":>8} {"last-first":>11}   (deg; mean over the dwell)')
    m = {}
    for j in JOINTS:
        f, l = np.mean(acc['first'][j]), np.mean(acc['last'][j])
        m[j] = (f, l)
        print(f'  {j:<22} {math.degrees(f):>8.2f} {math.degrees(l):>8.2f} {math.degrees(l - f):>+11.2f}')
    hips = [(m['left_hip_yaw_joint'][1] - m['left_hip_yaw_joint'][0]),
            (m['right_hip_yaw_joint'][1] - m['right_hip_yaw_joint'][0])]
    print(f'  mean hip-yaw change   {math.degrees(np.mean(hips)):+.2f} deg')


if __name__ == '__main__':
    main()
