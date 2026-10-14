#!/usr/bin/env python3
"""Start-offset ensemble: how sensitive is a result to where the replay begins?

A single deterministic replay is ONE draw. On live_walk_01, trimming 0..15 s off the
stationary start moved DR-on loop closure between 0.065 m and 1.246 m. So no closure
number means anything until it is shown as a distribution. This tool produces one.

For each offset it writes a trimmed copy of <bag>_plio (messages with header stamp
>= t0 + offset, stored order and receive stamps preserved), then runs the UNCHANGED
deterministic harness (tools/run_point_lio_det.sh) on it, so the Point-LIO parameters
cannot drift from the ones every other result uses. DR parameters are PINNED here and
recorded in the output, so a stray DR_* variable in the caller's shell cannot silently
change one arm of the on/off comparison.

Runs go in parallel, each in its own ROS_DOMAIN_ID with ROS_LOCALHOST_ONLY=1: replays
cannot cross-talk, and nothing on the robot's LAN can leak into a recording.

WHAT THE OFFSETS ARE AND ARE NOT (audited 2026-10-13):
  - They are perturbations of cold-start initialisation length on ONE walk. They are
    not independent draws, and k/n pair counts are evidence about this bag only.
    Generalising needs more bags, not more offsets.
  - They are NOT the live condition: live ran with ~20 s of warm pre-roll before the
    recorder started. Do not read "live landed in the good part of the spread".
  - Closure is BIMODAL (the run either re-registers to the start map or slips), so it is
    reported as slip rate + within-mode medians, not as one median/min/max band.

Metrics. Paired "DR on better" tallies use only metrics that measure error:
  closure       drift between first- and last-dwell mean positions (--loop only; real
                ground truth only if the walk started and ended on a marked spot)
  cl_tilt       how far world-up moved between the dwells; truth ~0
  cl_heading    heading change about world z (CCW +). Taken from the world-frame delta
                rotation: the IMU frame sits at pitch ~-85 deg, where ZYX yaw is garbage
  dz_walk_mean  mean |z - z_ref| while walking. The floor is flat, so truth is a few cm
  dz_walk_max   of gait bob. z_ref is the first-dwell mean (or the median of the first
                2 s on non-loop bags), NOT the first recorded pose: the recorder drops a
                variable 0.3-2.2 s of leading poses, which used to move z_end by ~9 mm.
Descriptive only, never tallied:
  peak_speed    real chest motion reaches ~0.84 m/s, so this is not an error measure
  legodom_agr   circular; on loop bags it is ANTI-correlated with truth
  head_lost     seconds of leading poses the recorder missed

Usage:
  tools/offset_sweep.py data/bags/<bag> [--loop] [--offsets 0,3,6,9,12,15]
                        [--dr 1,0] [--jobs 6] [--keep] [--slip 0.3]
Writes results/sweep_<bag>.json.
"""
import argparse, json, math, os, queue, shutil, struct, subprocess, sys
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import rosbag2_py
from eval_loop_closure import load_full, dwells, mean_pose, rel_rot
from dr_payoff import EST_TOPIC, REF_TOPIC

CLOUD = '/x2/points'
FEED = (CLOUD, '/aima/hal/sensor/lidar_chest_front/imu', REF_TOPIC)
# pinned DR parameters: identical to x2_ultra.yaml and run_point_lio_det.sh defaults
DR_PARAMS = {'DR_VEL_COV': '0.01', 'DR_MIN_DT': '0.0', 'DR_MAX_SPEED': '1.5'}


def stamp_ns(raw):
    # every fed type (PointCloud2, Imu, Odometry) starts with std_msgs/Header; after the
    # 4-byte CDR encapsulation header come int32 sec, uint32 nanosec. Verified equal to
    # the deserialised stamp on 40 000/40 000 messages across two bags.
    s, ns = struct.unpack_from('<iI', raw, 4)
    return s * 1_000_000_000 + ns


def read_feed(plio):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=plio, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    topics = [t for t in r.get_all_topics_and_types() if t.name in FEED]
    r.set_filter(rosbag2_py.StorageFilter(topics=[t.name for t in topics]))
    msgs = []
    while r.has_next():
        tn, raw, recv = r.read_next()
        msgs.append((stamp_ns(raw), tn, raw, recv))
    return topics, msgs


def write_trim(dst, topics, msgs, cutoff_ns):
    shutil.rmtree(dst, ignore_errors=True)
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=dst, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for t in topics:
        w.create_topic(rosbag2_py.TopicMetadata(
            name=t.name, type=t.type, serialization_format='cdr'))
    n = clouds = 0
    first_cloud = None
    for h, tn, raw, recv in msgs:          # stored order kept, exactly as the harness feeds
        if h >= cutoff_ns:
            w.write(tn, raw, recv)
            n += 1
            if tn == CLOUD:
                clouds += 1
                first_cloud = h if first_cloud is None else min(first_cloud, h)
    del w
    return n, clouds, first_cloud


# DDS domain ids are a pool of reusable slots, one per concurrent job. Fast DDS cannot
# compute ports for ids above 232, so handing out sequential ids per job would silently
# fail every run past ~130 jobs (each such node exits before replaying anything).
DOMAINS = queue.Queue()


def run_one(job):
    dom = DOMAINS.get()
    try:
        env = dict(os.environ, ROS_DOMAIN_ID=str(dom), ROS_LOCALHOST_ONLY='1',
                   DR_MODE=str(job['dr']), **DR_PARAMS)
        p = subprocess.run([os.path.join(HERE, 'run_point_lio_det.sh'), job['src'], job['out']],
                           env=env, cwd=ROOT, capture_output=True, text=True)
    finally:
        DOMAINS.put(dom)
    job['rc'] = p.returncode
    job['fed_clouds'] = None
    job['complete'] = False
    job['dr_applied'] = None
    log = os.path.join(ROOT, 'results', job['out'] + '.log')
    if os.path.exists(log):
        for line in open(log, errors='replace'):
            if 'bag exhausted:' in line:
                try:
                    job['fed_clouds'] = int(line.split('bag exhausted:')[1].split()[0])
                except ValueError:
                    pass
            if 'replay complete' in line:
                job['complete'] = True
            if '[dr] updates applied:' in line:
                try:
                    job['dr_applied'] = int(line.split('updates applied:')[1].split()[0])
                except ValueError:
                    pass
    return job


def evaluate(job, loop, ref, ref_dwells):
    rT, rP, rQ, rS = ref
    # a crashed or truncated replay must be an ERROR, never a quietly-scored row: a mode
    # that fails more often would otherwise lose its worst runs from the distribution.
    if job['fed_clouds'] != job['expect_clouds'] or not job['complete']:
        return {'error': f"replay incomplete: fed {job['fed_clouds']} of "
                         f"{job['expect_clouds']} clouds, complete={job['complete']}"}
    # a DR mode that never applies an update is indistinguishable from DR off -- refuse it
    if job['dr'] != 0 and not job.get('dr_applied'):
        return {'error': f"DR mode {job['dr']} applied {job.get('dr_applied')} updates"}
    res = os.path.join(ROOT, 'results', job['out'])
    try:
        eT, eP, eQ, _ = load_full(res, EST_TOPIC)
    except Exception as e:
        return {'error': f'no trajectory: {e}'}
    if len(eT) < 20:
        return {'error': f'only {len(eT)} poses'}
    if any(not math.isfinite(c) for p in eP for c in p):
        return {'error': 'non-finite pose'}
    out = {'poses': len(eT), 'head_lost': eT[0] - job['first_cloud'] / 1e9}

    sp = [math.dist(eP[i - 1], eP[i]) / (eT[i] - eT[i - 1])
          for i in range(1, len(eT)) if eT[i] > eT[i - 1]]
    out['peak_speed'] = max(sp)

    # height reference and walking segment
    if loop and len(ref_dwells) >= 2:
        d0, d1 = ref_dwells[0], ref_dwells[-1]
        zs = [p[2] for t, p in zip(eT, eP) if d0[0] <= t <= d0[1]]
        if not zs:
            return {**out, 'error': 'first dwell fully trimmed away'}
        z_ref = sum(zs) / len(zs)
        ws, we = d0[1], d1[0]
    else:
        early = sorted(p[2] for t, p in zip(eT, eP) if t <= eT[0] + 2.0)
        z_ref = early[len(early) // 2]
        ws, we = eT[0], eT[-1]
    dz = [abs(p[2] - z_ref) for t, p in zip(eT, eP) if ws <= t <= we]
    out['dz_walk_mean'] = sum(dz) / len(dz)
    out['dz_walk_max'] = max(dz)

    # circular metric, descriptive only
    i0 = next(i for i, t in enumerate(rT) if t >= eT[0])
    i1 = max(i for i, t in enumerate(rT) if t <= eT[-1])
    out['legodom_agr'] = abs(math.dist(eP[0], eP[-1]) - math.dist(rP[i0], rP[i1]))

    if loop and len(ref_dwells) >= 2:
        p0, M0 = mean_pose(eT, eP, eQ, *ref_dwells[0])
        p1, M1 = mean_pose(eT, eP, eQ, *ref_dwells[-1])
        if p0 is not None and p1 is not None:
            out['closure'] = math.dist(p0, p1)
            out['cl_z'] = p1[2] - p0[2]
            out['cl_heading'], out['cl_tilt'], _, _ = rel_rot(M0, M1)
        else:
            out['error'] = 'no poses in last dwell'
    return out


def med(xs):
    xs = sorted(xs)
    n = len(xs)
    return None if not n else (xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--loop', action='store_true',
                    help='walk started and ended on the same marked spot')
    ap.add_argument('--offsets', default='0,3,6,9,12,15')
    ap.add_argument('--dr', default='1,0')
    ap.add_argument('--jobs', type=int, default=6)
    ap.add_argument('--slip', type=float, default=0.3,
                    help='closure above this (m) counts as a slip (bad mode)')
    ap.add_argument('--keep', action='store_true', help='keep trimmed bags')
    ap.add_argument('--tag', default='',
                    help='write results/sweep_<bag>_<tag>.json instead of overwriting the '
                         'baseline sweep (e.g. --dr 3,4 --tag modes34)')
    a = ap.parse_args()

    bag = a.bag.rstrip('/')
    name = os.path.basename(bag)
    plio = bag + '_plio'
    if not os.path.isdir(plio):
        sys.exit(f'{plio} missing: run tools/run_point_lio_det.sh {bag} once to create it')
    offsets = [float(x) for x in a.offsets.split(',')]
    drs = [int(x) for x in a.dr.split(',')]

    print(f'==> reading {plio}')
    topics, msgs = read_feed(plio)
    t0 = min(m[0] for m in msgs)
    print(f'    {len(msgs)} fed messages, t0 = {t0 / 1e9:.3f}')

    suffix = f'_{a.tag}' if a.tag else ''
    sweep_dir = os.path.join(ROOT, 'data', 'bags', f'_sweep_{name}{suffix}')
    os.makedirs(sweep_dir, exist_ok=True)
    jobs = []
    for off in offsets:
        tag = f'{name}_t{off:g}'
        src = os.path.join(sweep_dir, tag)
        os.makedirs(src, exist_ok=True)          # harness only needs the dir to exist
        n, clouds, first_cloud = write_trim(src + '_plio', topics, msgs, t0 + int(off * 1e9))
        print(f'    trim {off:>4g} s -> {n} messages, {clouds} clouds')
        for dr in drs:
            jobs.append({'off': off, 'dr': dr, 'src': src, 'out': f'sweep_{tag}_dr{dr}',
                         'expect_clouds': clouds, 'first_cloud': first_cloud})
    del msgs

    if not 1 <= a.jobs <= 100:
        sys.exit('--jobs must be 1..100 (one DDS domain per concurrent job, ids 100..199)')
    for k in range(a.jobs):
        DOMAINS.put(100 + k)
    print(f'==> {len(jobs)} runs, {a.jobs} in parallel (isolated DDS domains)')
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        done = list(ex.map(run_one, jobs))

    ref = load_full(bag, REF_TOPIC)
    ref_dwells = dwells(ref[0], ref[3], 0.05, 3.0)
    if a.loop and len(ref_dwells) < 2:
        print('  WARNING: --loop given but fewer than two dwells found; closure unavailable')
    rows = []
    for j in done:
        m = evaluate(j, a.loop, ref, ref_dwells)
        rows.append({**{k: j.get(k) for k in ('off', 'dr', 'out', 'rc', 'fed_clouds',
                                              'expect_clouds', 'complete', 'dr_applied')}, **m})

    if not a.keep:
        shutil.rmtree(sweep_dir, ignore_errors=True)

    # ------------------------------------------------------------------ report
    loopcols = a.loop and len(ref_dwells) >= 2
    print(f'\n=== {name}: start-offset ensemble ===')
    hdr = (f"  {'trim':>5} {'DR':>3} {'poses':>6} {'lost s':>6} {'dz mean':>8} {'dz max':>7}"
           f" {'peak m/s':>9}")
    if loopcols:
        hdr += f" {'closure':>9} {'cl_z':>7} {'heading':>8} {'tilt':>6}"
    print(hdr)
    for r in sorted(rows, key=lambda r: (r['off'], -r['dr'])):
        if 'error' in r:
            print(f"  {r['off']:>5g} {r['dr']:>3}  ERROR: {r['error']}")
            continue
        line = (f"  {r['off']:>5g} {r['dr']:>3} {r['poses']:>6} {r['head_lost']:>6.2f}"
                f" {r['dz_walk_mean']:>7.3f}m {r['dz_walk_max']:>6.3f}m {r['peak_speed']:>9.3f}")
        if loopcols:
            line += (f" {r['closure']:>8.3f}m {r['cl_z']:>+7.3f} {r['cl_heading']:>+7.2f}d"
                     f" {r['cl_tilt']:>5.2f}d")
        print(line)

    ok = [r for r in rows if 'error' not in r]
    failed = [r for r in rows if 'error' in r]
    summary = {}
    print(f'\n=== per DR mode ===')
    for dr in drs:
        rs = [r for r in ok if r['dr'] == dr]
        nf = sum(1 for r in failed if r['dr'] == dr)
        s = {'n_ok': len(rs), 'n_failed': nf,
             'dz_walk_mean_median': med([r['dz_walk_mean'] for r in rs]),
             'dz_walk_max_median': med([r['dz_walk_max'] for r in rs])}
        line = (f"  DR {dr}: {len(rs)} ok, {nf} failed | dz_walk mean (median) "
                f"{s['dz_walk_mean_median']:.3f} m, max (median) {s['dz_walk_max_median']:.3f} m")
        if loopcols:
            cl = [r['closure'] for r in rs]
            good = [c for c in cl if c <= a.slip]
            bad = [c for c in cl if c > a.slip]
            s.update({'slip_rate': len(bad) / len(cl) if cl else None,
                      'closure_good_median': med(good), 'closure_bad_median': med(bad),
                      'tilt_median': med([r['cl_tilt'] for r in rs])})
            line += (f"\n         closure: slipped {len(bad)}/{len(cl)} (>{a.slip} m); "
                     f"good-mode median {s['closure_good_median'] or float('nan'):.3f} m, "
                     f"slip-mode median {s['closure_bad_median'] or float('nan'):.3f} m; "
                     f"tilt median {s['tilt_median']:.2f} deg")
        print(line)
        summary[dr] = s

    paired_metrics = ['dz_walk_mean', 'dz_walk_max'] + (['closure', 'cl_tilt'] if loopcols else [])
    if set(drs) >= {0, 1}:
        print(f'\n=== paired: same trim, DR on vs off (error metrics only; lower is better) ===')
        print(f'  (offsets are correlated perturbations of one walk: evidence about THIS bag only)')
        pairs = {}
        for m in paired_metrics:
            wins = losses = ties = missing = 0
            for off in offsets:
                on = next((r for r in ok if r['off'] == off and r['dr'] == 1 and m in r), None)
                of = next((r for r in ok if r['off'] == off and r['dr'] == 0 and m in r), None)
                if not (on and of):
                    missing += 1
                    continue
                if on[m] < of[m]: wins += 1
                elif on[m] > of[m]: losses += 1
                else: ties += 1
            pairs[m] = {'wins': wins, 'losses': losses, 'ties': ties, 'missing': missing}
            note = f', {missing} pair(s) MISSING (failed runs)' if missing else ''
            print(f'  {m:<13} DR on better in {wins}/{wins + losses + ties}{note}')
        summary['paired'] = pairs

    if failed:
        print(f'\n  WARNING: {len(failed)} run(s) failed -- excluded and counted above, see results/*.log')

    path = os.path.join(ROOT, 'results', f'sweep_{name}{suffix}.json')
    json.dump({'bag': name, 'loop': a.loop, 'offsets': offsets, 'slip_threshold': a.slip,
               'dr_params': DR_PARAMS, 'rows': rows,
               'summary': {str(k): v for k, v in summary.items()}}, open(path, 'w'), indent=1)
    print(f'\n  wrote {os.path.relpath(path, ROOT)}')


if __name__ == '__main__':
    main()
