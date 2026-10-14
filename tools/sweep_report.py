#!/usr/bin/env python3
"""Cross-bag summary of tools/offset_sweep.py results.

Usage: tools/sweep_report.py [results/sweep_<bag>*.json ...]
       (default: every results/sweep_*.json that is not an archived _vN copy)

Files are merged per bag, so a baseline sweep (--dr 1,0) and an experiment sweep run later
with --tag (--dr 3,4) appear as one block. Merging is only valid because the deterministic
harness reproduces a mode bit-for-bit across builds that leave that mode's code unchanged
-- re-verify that whenever the filter is rebuilt.

Every mode is compared run-by-run (same start offset) against mode 0 (no DR) and mode 1
(the current DR factor), using only the metrics offset_sweep.py marks as error measures.
Bags are the unit of generalisation: offsets within a bag are correlated.
"""
import glob, json, os, re, sys
from collections import OrderedDict

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODE_NAME = {0: 'off', 1: 'DR1', 2: 'DR2', 3: 'DR3 xy', 4: 'DR4 jac'}


def med(xs):
    xs = sorted(x for x in xs if x is not None)
    n = len(xs)
    return None if not n else (xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2]))


def fmt(x, unit='m', w=7):
    return f"{'--':>{w}}" if x is None else f"{x:>{w-1}.3f}{unit}"


def paired(rows, m_new, m_ref, metric):
    by = {}
    for r in rows:
        if 'error' not in r and metric in r:
            by[(r['off'], r['dr'])] = r[metric]
    offs = sorted({o for o, _ in by})
    w = l = 0
    for o in offs:
        if (o, m_new) in by and (o, m_ref) in by:
            w += by[(o, m_new)] < by[(o, m_ref)]
            l += by[(o, m_new)] > by[(o, m_ref)]
    return w, w + l


def main():
    paths = sys.argv[1:] or sorted(p for p in glob.glob(os.path.join(ROOT, 'results', 'sweep_*.json'))
                                   if not re.search(r'_v\d+\.json$', p))
    bags = OrderedDict()
    for p in paths:
        d = json.load(open(p))
        b = bags.setdefault(d['bag'], {'rows': [], 'loop': False, 'thr': d.get('slip_threshold', 0.3),
                                       'offsets': set(), 'files': []})
        b['rows'] += d['rows']
        b['loop'] |= bool(d.get('loop'))
        b['offsets'] |= set(d['offsets'])
        b['files'].append(os.path.basename(p))

    for name, b in bags.items():
        rows = b['rows']
        loop = b['loop'] and any('closure' in r for r in rows)
        modes = sorted({r['dr'] for r in rows}, key=lambda m: (m != 0, m))
        print(f"\n### {name}  ({'marked loop' if loop else 'no ground truth'}, "
              f"{len(b['offsets'])} start offsets; {', '.join(b['files'])})")
        hdr = f"  {'mode':<8} {'ok':>3} {'fail':>4}  {'dz mean':>8} {'dz max':>8} {'peak m/s':>9}"
        if loop:
            hdr += f"  {'slips':>6} {'closure good':>12} {'tilt':>7}"
        print(hdr)
        for m in modes:
            rs = [r for r in rows if r['dr'] == m]
            ok = [r for r in rs if 'error' not in r]
            line = (f"  {MODE_NAME.get(m, m):<8} {len(ok):>3} {len(rs)-len(ok):>4}  "
                    f"{fmt(med([r['dz_walk_mean'] for r in ok]), 'm', 8)} "
                    f"{fmt(med([r['dz_walk_max'] for r in ok]), 'm', 8)} "
                    f"{max((r['peak_speed'] for r in ok), default=float('nan')):>9.2f}")
            if loop:
                cl = [r['closure'] for r in ok if 'closure' in r]
                good = [c for c in cl if c <= b['thr']]
                line += (f"  {sum(c > b['thr'] for c in cl):>2}/{len(cl):<3} {fmt(med(good), 'm', 12)} "
                         f"{fmt(med([r['cl_tilt'] for r in ok if 'cl_tilt' in r]), 'd', 7)}")
            print(line)
        metrics = ['dz_walk_mean', 'dz_walk_max'] + (['closure', 'cl_tilt'] if loop else [])
        for ref in (0, 1):
            if ref not in modes:
                continue
            for m in modes:
                if m == ref or (ref == 1 and m == 0):
                    continue
                bits = []
                for k in metrics:
                    w, n = paired(rows, m, ref, k)
                    if n:
                        bits.append(f"{k} {w}/{n}")
                if bits:
                    print(f"  {MODE_NAME.get(m, m)} better than {MODE_NAME[ref]}: " + ', '.join(bits))
    print("\n  dz = |height - start height| while walking (flat floor: truth is a few cm of gait).")
    print("  slips = runs whose loop closure exceeded the threshold (bimodal outcome).")
    print("  fail = replay did not complete (crash/abort); counted, never silently dropped.")


if __name__ == '__main__':
    main()
