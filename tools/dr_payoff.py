#!/usr/bin/env python3
"""Bound the payoff of a leg-odometry dead-reckoning factor, offline.

This does NOT implement a DR factor. It asks a cheaper question: if Point-LIO's
per-scan translation *magnitude* had been constrained by leg odometry, how much
of the measured drift would disappear?

Method: Point-LIO gives a direction per 100 ms step, leg odometry gives a
magnitude. Where the two disagree beyond a gate, rescale the step to the leg
odometry magnitude, keep Point-LIO's direction, and re-integrate.

Only magnitudes are compared, so the unknown yaw offset between `leg_odom` and
`camera_init` never enters. The flip side: this cannot correct orientation
error, so it is a LOWER bound on what an in-filter DR factor would recover.

Usage: tools/dr_payoff.py [results/<run>] [data/bags/<bag>]
"""
import sys, math, bisect
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

V_MAX = 0.75   # m/s, X2 Ultra platform maximum
EST_TOPIC = '/aft_mapped_to_init'
REF_TOPIC = '/aima/mc/leg_odometry'


def load(uri, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    T, P = [], []
    while r.has_next():
        _, raw, _ = r.read_next()
        m = deserialize_message(raw, Odometry)
        T.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        P.append((m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z))
    return T, P


def interp(T, P, t):
    """Linearly interpolate position at time t (clamped to the endpoints)."""
    if t <= T[0]:
        return P[0]
    if t >= T[-1]:
        return P[-1]
    i = bisect.bisect_left(T, t)
    t0, t1 = T[i - 1], T[i]
    a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
    return tuple(P[i - 1][k] + a * (P[i][k] - P[i - 1][k]) for k in range(3))


def integrate(deltas):
    p = (0.0, 0.0, 0.0)
    out = [p]
    for d in deltas:
        p = (p[0] + d[0], p[1] + d[1], p[2] + d[2])
        out.append(p)
    return out


def norm(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def main():
    est_uri = sys.argv[1] if len(sys.argv) > 1 else 'results/corridor_det_1'
    ref_uri = sys.argv[2] if len(sys.argv) > 2 else 'data/bags/office_corridor_02'

    est_t, est_p = load(est_uri, EST_TOPIC)
    ref_t, ref_p = load(ref_uri, REF_TOPIC)

    # per-step: Point-LIO delta, and the reference displacement over the same window
    steps = []
    for i in range(1, len(est_t)):
        dt = est_t[i] - est_t[i - 1]
        de = tuple(est_p[i][k] - est_p[i - 1][k] for k in range(3))
        r0 = interp(ref_t, ref_p, est_t[i - 1])
        r1 = interp(ref_t, ref_p, est_t[i])
        dr = tuple(r1[k] - r0[k] for k in range(3))
        steps.append((est_t[i], dt, de, norm(de), norm(dr)))

    span = est_t[-1] - est_t[0]
    ref_path = sum(s[4] for s in steps)
    est_path = sum(s[3] for s in steps)

    # the robot physically walked back to near its start; find that moment
    t0 = ref_t[0]
    probe_d, probe_rel = min((norm(tuple(ref_p[i][k] - ref_p[0][k] for k in range(3))),
                              ref_t[i] - t0)
                             for i in range(len(ref_p)) if ref_t[i] - t0 > 30)
    probe_t = t0 + probe_rel
    probe_i = min(range(len(steps)), key=lambda i: abs(steps[i][0] - probe_t))

    print(f"=== inputs ===")
    print(f"  estimate  {est_uri}   {len(est_t)} poses, {span:.1f} s")
    print(f"  reference {ref_uri}   {len(ref_t)} poses")
    print(f"  path length: leg odometry {ref_path:.1f} m, Point-LIO {est_path:.1f} m "
          f"({est_path / ref_path:.2f}x)")
    print(f"  return-to-start probe: reference is {probe_d:.2f} m from start "
          f"at t={probe_rel:.1f} s")

    def report(name, deltas, n_fixed, removed):
        traj = integrate(deltas)
        end = norm(traj[-1])
        at_probe = norm(traj[probe_i + 1])
        err = abs(at_probe - probe_d)
        print(f"  {name:<26} {end:7.2f} {at_probe:8.2f} {err:8.2f}   "
              f"{n_fixed:5d} {removed:8.2f}")
        return err

    print(f"\n=== payoff sweep ===")
    print(f"  {'variant':<26} {'end_d':>7} {'at_probe':>8} {'err':>8}   "
          f"{'fixed':>5} {'m_removed':>8}")

    baseline = report('baseline (no DR)', [s[2] for s in steps], 0, 0.0)

    # gate A: cap any step that implies more than the platform's maximum speed
    for k in (1.0, 1.5, 2.0, 3.0):
        deltas, n, removed = [], 0, 0.0
        for _, dt, de, d_est, d_ref in steps:
            cap = k * V_MAX * dt
            if d_est > cap and d_est > 1e-9:
                deltas.append(tuple(c * cap / d_est for c in de))
                n += 1
                removed += d_est - cap
            else:
                deltas.append(de)
        report(f'speed cap {k:.1f}x v_max', deltas, n, removed)

    # gate B: rescale to the leg-odometry magnitude when they disagree by > K
    for K in (1.5, 2.0, 3.0, 5.0):
        deltas, n, removed = [], 0, 0.0
        for _, dt, de, d_est, d_ref in steps:
            if d_est > K * d_ref + 0.01 and d_est > 1e-9:
                deltas.append(tuple(c * d_ref / d_est for c in de))
                n += 1
                removed += d_est - d_ref
            else:
                deltas.append(de)
        report(f'ratio gate {K:.1f}x leg_odom', deltas, n, removed)

    # gate C: trust leg odometry for every step's magnitude (pure scale-DR)
    deltas, removed = [], 0.0
    for _, dt, de, d_est, d_ref in steps:
        if d_est > 1e-9:
            deltas.append(tuple(c * d_ref / d_est for c in de))
            removed += d_est - d_ref
        else:
            deltas.append(de)
    report('always (pure scale-DR)', deltas, len(steps), removed)

    # where does the baseline error actually come from?
    print(f"\n=== worst steps by excess over leg odometry ===")
    print(f"  {'t_rel':>7} {'dt':>6} {'d_est':>7} {'d_ref':>7} {'excess':>7} {'v_est':>7}")
    worst = sorted(steps, key=lambda s: s[3] - s[4], reverse=True)[:10]
    for t, dt, de, d_est, d_ref in worst:
        print(f"  {t - est_t[0]:7.1f} {dt:6.3f} {d_est:7.3f} {d_ref:7.3f} "
              f"{d_est - d_ref:7.3f} {d_est / dt:7.2f}")
    tot_excess = sum(max(0.0, s[3] - s[4]) for s in steps)
    top10 = sum(s[3] - s[4] for s in worst)
    print(f"  top 10 steps hold {top10:.2f} m of {tot_excess:.2f} m total excess "
          f"({100 * top10 / tot_excess:.0f} %)")
    print(f"\n  baseline return-to-start error: {baseline:.2f} m")


if __name__ == '__main__':
    main()
