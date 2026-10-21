#!/usr/bin/env python3
"""Does the robot do what the remote commands? Mean `rc` command over 1 s windows vs leg-odometry
WORLD motion (net displacement / dt, heading change / dt) over the same window shifted by a lag.

Prints, per lag, corr(cmd fwd, speed) and corr(cmd ang, yaw rate), then binned tables at the lag
with the best speed correlation. The stick is rarely steady, so windows mix accelerations: read
the bins as "roughly", and low-speed bins have few samples.

Usage: tools/cmd_tracking.py data/bags/<bag> [window_s]
"""
import sys, math, bisect
import numpy as np
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
from decode_locomotion_cmd import read, parse, TOPIC
from eval_loop_closure import load_full, rotmat, rel_rot


def main():
    bag = sys.argv[1]; W = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    msgs = [parse(raw) for _, raw, _ in read(bag, TOPIC)]
    srcs = {}
    for m in msgs:
        srcs.setdefault(m['source'], []).append(m)
    for s, ms in srcs.items():
        nz = [m for m in ms if abs(m['fwd']) + abs(m['ang']) + abs(m['lat']) > 1e-9]
        print(f'  source {s!r}: {len(ms)} msgs, {len(nz)} nonzero, fwd max {max(m["fwd"] for m in ms):.3f}')
    msgs = srcs.get('rc', [])
    if not msgs:
        sys.exit('no rc commands in this bag')
    cT = np.array([m['t'] for m in msgs]); cF = np.array([m['fwd'] for m in msgs]); cA = np.array([m['ang'] for m in msgs])
    lT, lP, lQ, _ = load_full(bag, '/aima/mc/leg_odometry')
    lT = np.array(lT); lP = np.array(lP)

    def motion(t0, t1):
        a = bisect.bisect_left(lT, t0); b = min(bisect.bisect_left(lT, t1), len(lT) - 1)
        if b <= a + 5:
            return None
        dt = lT[b] - lT[a]
        h, *_ = rel_rot(rotmat(lQ[a]), rotmat(lQ[b]))
        return math.hypot(*(lP[b][:2] - lP[a][:2])) / dt, math.radians(h) / dt

    best = None
    for lag in (0.0, 0.25, 0.5, 0.75, 1.0, 1.5):
        rows = []; t = cT[0]
        while t + W < cT[-1]:
            i, j = bisect.bisect_left(cT, t), bisect.bisect_left(cT, t + W)
            if j - i > 10:
                m = motion(t + lag, t + W + lag)
                if m:
                    rows.append((cF[i:j].mean(), cA[i:j].mean(), m[0], m[1]))
            t += W
        r = np.array(rows); mv = (abs(r[:, 0]) > 0.01) | (abs(r[:, 1]) > 0.02)
        cf, ca = np.corrcoef(r[mv, 0], r[mv, 2])[0, 1], np.corrcoef(r[mv, 1], r[mv, 3])[0, 1]
        print(f'lag {lag:.2f} s: {mv.sum()} moving windows  corr(fwd,speed) {cf:+.2f}  corr(ang,yawrate) {ca:+.2f}')
        if best is None or cf > best[0]:
            best = (cf, lag, r)
    cf, lag, r = best
    print(f'best lag {lag} s; binned (|cmd ang| < 0.1 for speed; |cmd fwd| < 0.05 for yaw rate):')
    for lo, hi in ((0.0, 0.02), (0.02, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.45), (0.45, 0.6), (0.6, 0.9)):
        s = r[(r[:, 0] >= lo) & (r[:, 0] < hi) & (abs(r[:, 1]) < 0.1)]
        if len(s):
            print(f'   cmd fwd [{lo:.2f},{hi:.2f})  n {len(s):4d}  speed median {np.median(s[:, 2]):.3f}  '
                  f'[p10 {np.percentile(s[:, 2], 10):.3f}, p90 {np.percentile(s[:, 2], 90):.3f}]')
    for lo, hi in ((-1.0, -0.3), (-0.3, -0.1), (-0.1, -0.02), (0.02, 0.1), (0.1, 0.3), (0.3, 1.0)):
        s = r[(r[:, 1] >= lo) & (r[:, 1] < hi) & (abs(r[:, 0]) < 0.05)]
        if len(s):
            print(f'   cmd ang [{lo:+.2f},{hi:+.2f})  n {len(s):4d}  yaw rate median {np.median(s[:, 3]):+.3f}  '
                  f'[p10 {np.percentile(s[:, 3], 10):+.3f}, p90 {np.percentile(s[:, 3], 90):+.3f}]')


if __name__ == '__main__':
    main()
