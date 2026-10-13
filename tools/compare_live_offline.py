#!/usr/bin/env python3
"""Compare a LIVE on-robot trajectory against a deterministic offline replay of the
same recorded inputs.

Everything else in this project is measured offline. This is the one check that says
whether the live path -- real DDS, real scheduling, real contention -- produces the same
answer as the harness. If it does, offline results transfer. If it does not, the
difference is what DDS and timing cost us.

Usage: tools/compare_live_offline.py data/bags/<bag> results/<offline_run>
"""
import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_loop_closure import load_full
from dr_payoff import norm, EST_TOPIC


def main():
    bag, off = sys.argv[1], sys.argv[2]
    lT, lP, _, _ = load_full(bag, EST_TOPIC)
    oT, oP, _, _ = load_full(off, EST_TOPIC)
    print(f"live    {len(lT)} poses from {bag}")
    print(f"offline {len(oT)} poses from {off}")

    # match on exact header stamps: both derive from the same cloud timestamps
    lm = {round(t, 6): p for t, p in zip(lT, lP)}
    om = {round(t, 6): p for t, p in zip(oT, oP)}
    common = sorted(set(lm) & set(om))
    print(f"common stamps: {len(common)}")
    if not common:
        print("  NO common stamps -- cannot compare pose by pose")
        return 1

    d = [math.dist(lm[t], om[t]) for t in common]
    d.sort()
    n = len(d)
    print(f"\n=== per-pose divergence (live vs offline, same inputs) ===")
    print(f"  identical : {sum(1 for x in d if x == 0.0)}/{n}")
    print(f"  median    : {d[n//2]:.4f} m")
    print(f"  p90       : {d[int(0.90*n)]:.4f} m")
    print(f"  p99       : {d[int(0.99*n)]:.4f} m")
    print(f"  max       : {d[-1]:.4f} m")

    # endpoint divergence matters more than any single pose
    le = tuple(lm[common[-1]][k] - lm[common[0]][k] for k in range(3))
    oe = tuple(om[common[-1]][k] - om[common[0]][k] for k in range(3))
    print(f"\n  net displacement live    : {norm(le):.4f} m")
    print(f"  net displacement offline : {norm(oe):.4f} m")
    print(f"  endpoint difference      : {abs(norm(le)-norm(oe)):.4f} m")
    print(f"\n  Interpretation: Point-LIO is iterative and the live run is fed by DDS with")
    print(f"  real scheduling, so bit-identity is NOT expected. What matters is whether the")
    print(f"  divergence stays far below the measurement floor (~5 cm repositioning).")
    return 0


if __name__ == '__main__':
    sys.exit(main())
