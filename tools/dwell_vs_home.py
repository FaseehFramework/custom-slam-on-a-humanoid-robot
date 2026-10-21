#!/usr/bin/env python3
"""Where do a walk's first and last stand-stills land on the map, relative to HOME?

HOME = the mapping walk's first stand-still (the taped mark), in the map frame. A test
walk that starts and ends on the same mark should put both of its stand-stills on HOME,
up to the operator's repositioning error. Offsets are given as forward / left of HOME's
heading (robot-forward measured from leg odometry, see closure_in_robot_frame.py).

Usage: tools/dwell_vs_home.py <home_traj> <home_data_bag> <walk_traj> <walk_data_bag>
  traj bags: map-frame /aft_mapped_to_init (pgo `final`, localizer `traj`)
"""
import sys, os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from eval_loop_closure import load_full, dwells, mean_pose
from closure_in_robot_frame import body_forward
from dr_payoff import EST_TOPIC, REF_TOPIC


def dwell_poses(traj, bag):
    rT, _, _, rS = load_full(bag, REF_TOPIC)
    d = dwells(rT, rS, 0.05, 3.0)
    eT, eP, eQ, _ = load_full(traj, EST_TOPIC)
    out = []
    for a, b in (d[0], d[-1]):
        p, M = mean_pose(eT, eP, eQ, a, b)
        out.append((np.array(p), M, a - rT[0], b - rT[0]))
    return out


def main():
    ht, hb, wt, wb = sys.argv[1:5]
    f_body, _, _ = body_forward(hb)
    (hp, hM, _, _), _ = dwell_poses(ht, hb)
    fw = hM @ f_body; fw[2] = 0; fw /= np.linalg.norm(fw)
    left = np.cross([0, 0, 1.0], fw)
    print(f'HOME (first dwell of {os.path.basename(hb)}) at x {hp[0]:+.3f} y {hp[1]:+.3f} z {hp[2]:+.3f}')
    for name, (p, M, a, b) in zip(('start', 'end'), dwell_poses(wt, wb)):
        d = p - hp
        D = M @ hM.T                      # world-frame delta rotation (rel_rot convention)
        head = math.degrees(math.atan2(D[1, 0], D[0, 0]))
        print(f'  walk {name:<5} ({a:5.1f}-{b:5.1f} s): {np.linalg.norm(d[:2]):.3f} m from HOME = '
              f'forward {d @ fw:+.3f}  left {d @ left:+.3f}  dz {d[2]:+.3f} m   heading {head:+.2f} deg')


if __name__ == '__main__':
    main()
