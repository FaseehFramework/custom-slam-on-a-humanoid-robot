#!/usr/bin/env python3
"""Two checks the waypoint follower's steering rests on, on one walk:

1. Turn direction: leg odometry's world frame vs the map-frame pose agree on the SIGN of
   heading change (2 s chunks turning > 10 deg). With tools/cmd_tracking.py (command vs
   leg-odometry yaw rate) this ties `angular_velocity > 0` to counter-clockwise in the map.
2. Robot heading from the pose: R_map_body * f_body (f_body = robot-forward in the IMU frame,
   from leg-odometry walking velocity, as closure_in_robot_frame.py) vs the direction of
   travel in the map, on straight (< 5 deg turn), brisk (> 0.3 m/s) 2 s spans. Also through
   the waypoint follower's own heading_from_quat + F_BODY: a wrong constant shows up here.

Usage: tools/heading_check.py data/bags/<bag> <map_pose_bag> [topic]
       (e.g. results/loc01_live/live_loc, whose /aft_mapped_to_init is the map-frame pose)
"""
import sys, math, bisect
import numpy as np
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from eval_loop_closure import load_full, rotmat, rel_rot
from closure_in_robot_frame import body_forward


def main():
    bag, loc = sys.argv[1], sys.argv[2]
    topic = sys.argv[3] if len(sys.argv) > 3 else '/aft_mapped_to_init'
    f_body, n, cons = body_forward(bag)
    print(f'robot-forward in body frame {np.round(f_body, 3)} ({n} walking samples, consistency {cons:.2f})')
    lT, lP, lQ, _ = load_full(bag, '/aima/mc/leg_odometry')
    mT, mP, mQ, _ = load_full(loc, topic)
    lT, mT, mP = np.array(lT), np.array(mT), np.array(mP)
    at = lambda T, t: min(bisect.bisect_left(T, t), len(T) - 1)
    a_, b_ = [], []
    t = mT[0]
    while t + 2.0 < mT[-1]:
        hl, *_ = rel_rot(rotmat(lQ[at(lT, t)]), rotmat(lQ[at(lT, t + 2.0)]))
        hm, *_ = rel_rot(rotmat(mQ[at(mT, t)]), rotmat(mQ[at(mT, t + 2.0)]))
        if abs(hm) > 10:
            a_.append(hl); b_.append(hm)
        t += 2.0
    a_, b_ = np.array(a_), np.array(b_)
    print(f'1. 2 s chunks turning > 10 deg: {len(a_)}; same sign {np.sum(np.sign(a_) == np.sign(b_))}; '
          f'median |leg - map| {np.median(abs(a_ - b_)):.1f} deg; corr {np.corrcoef(a_, b_)[0, 1]:+.3f}')
    import follower_core as fc
    for label, f in (('this walk\'s f_body', f_body), ('follower_core.F_BODY', np.array(fc.F_BODY))):
        f = f / np.linalg.norm(f)
        errs = []
        for j in range(0, len(mT) - 20, 5):
            k = j + 20
            d = mP[k, :2] - mP[j, :2]
            h, *_ = rel_rot(rotmat(mQ[j]), rotmat(mQ[k]))
            if np.linalg.norm(d) / (mT[k] - mT[j]) < 0.3 or abs(h) > 5:
                continue
            q = mQ[(j + k) // 2]
            if label.startswith('follower'):      # exactly the follower's own function
                e = fc.heading_from_quat(*q) - math.atan2(d[1], d[0])
            else:
                fw = rotmat(q) @ f
                e = math.atan2(fw[1], fw[0]) - math.atan2(d[1], d[0])
            errs.append(math.degrees((e + math.pi) % (2 * math.pi) - math.pi))
        errs = np.array(errs)
        print(f'2. [{label}] straight 2 s spans: {len(errs)}; heading(pose) - heading(travel): median '
              f'{np.median(errs):+.1f} deg, p10 {np.percentile(errs, 10):+.1f}, p90 {np.percentile(errs, 90):+.1f}')


if __name__ == '__main__':
    main()
