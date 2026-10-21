#!/usr/bin/env python3
"""Offline tests for the waypoint follower: the REAL run_route() (tools/follower_core.py)
driving a simulated X2 + mc, in virtual time. No ROS, no robot. ~1 min.

The simulated mc follows what was measured (README "How mc arbitrates", "Steering facts"):
  - stand -> walk is mc's own switch on an accepted non-zero command, after `start_delay`
    (0.23 s measured; 2.6 s behind an animation); back to stand 3.6 s after commands end
  - a source's command expires 1 s after its last message (mc's own watchdog). The whole
    scenario list ALSO runs with that expiry switched off (mc_expiry=None), so the follower's
    own zero tail must stop the robot by itself.
  - the remote (rc, priority 80) streams ~50 Hz zeros while connected; a non-zero rc
    command wins over pnc
  - mc's walking policy, as MEASURED on the robot (README "Stage C1", follow_c_01):
      in place: |ang| < 0.45 -> no turn; 0.45-0.5 -> 0.11 rad/s; >= 0.5 -> w_inplace (0.5-1.0);
                after the command drops, it carries on ~0.25-0.4 s (yaw lag + yaw tau)
      walking:  turns track the command (walk_turn_gain ~1.1)
      forward:  >= 0.12 starts walking (>= 0.1 keeps it): creep_s of ~0.05 m/s, then
                max(v_walk ~0.45, 0.9 x cmd); below 0.1: creeping at half the command
    The earlier LINEAR model encoded the very assumption C1 disproved; it is gone.
  - forward command -> motion lag ~0.3 s, first-order speed response
  - angular > 0 = counter-clockwise; chest IMU ~0.13 m ahead of the turning centre
  - gait sway on heading (4 deg) and chest (3 cm) while walking; pose 10 Hz, status 1 Hz
The sim's IMU mounting uses its OWN literal for robot-forward (F_TRUE), not fc.F_BODY.

Invariants checked on EVERY run, against the rule's literal numbers (CLAUDE.md #15), not the
code's constants: 0 <= fwd <= 0.20, |ang| <= 0.30; REFUSED => nothing sent; otherwise the run
ends with a continuous >= 1 s zero tail (>= 15 messages, no gap > 0.1 s); the robot ends stopped.
Fault scenarios also bound WHEN the last non-zero command went out.

Usage: tools/test_follower.py [-v] [--random N] [-k substring]
"""
import argparse, math, random, sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follower_core as fc

RULE_FWD, RULE_ANG, RULE_LEG = 0.20, 0.60, 10.0     # CLAUDE.md rule 15, literally (turn cap 0.6 since 2026-10-21)
F_TRUE = (0.0158, -0.0213, -0.9992)                  # measured robot-forward (README), typed independently


def _mount(f):
    """R_mount (rows f, l, u): body -> world at yaw 0, with R_mount @ f = +x."""
    n = math.sqrt(sum(c * c for c in f)); f = [c / n for c in f]
    xb = (1.0, 0.0, 0.0)
    d = sum(a * b for a, b in zip(xb, f))
    u = [xb[i] - d * f[i] for i in range(3)]
    n = math.sqrt(sum(c * c for c in u)); u = [c / n for c in u]
    l = [u[1] * f[2] - u[2] * f[1], u[2] * f[0] - u[0] * f[2], u[0] * f[1] - u[1] * f[0]]
    return (tuple(f), tuple(l), tuple(u))


MOUNT = _mount(F_TRUE)


def mat_to_quat(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, 0.25 * s)
    i = max(range(3), key=lambda k: R[k][k])
    if i == 0:
        s = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2
        return (0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s, (R[2][1] - R[1][2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2
        return ((R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s, (R[0][2] - R[2][0]) / s)
    s = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2
    return ((R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s, (R[1][0] - R[0][1]) / s)


def quat_for_heading(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
    R = [[sum(Rz[i][k] * MOUNT[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mat_to_quat(R)


class Sim:
    """Simulated robot + mc + localiser + remote, advanced only by sleep()."""

    def __init__(self, seed=0, x=0.0, y=0.0, yaw=0.0, **kw):
        self.rng = random.Random(seed)
        self.t = 0.0
        self.x, self.y, self.yaw = x, y, yaw          # turning centre, true heading
        self.v = self.w = 0.0
        self.cfg = dict(start_delay=0.23, ang_sign=1.0, fwd_sign=1.0, tau=0.35, lag=0.3,
                        v_walk=0.45, creep_s=2.7, creep_v=0.05, w_inplace=0.95, inplace_on=0.5,
                        walk_turn_gain=1.1, yaw_lag=0.25, yaw_tau=0.12, v_cap=9.0, stuck=False, never_walk=False, r_chest=fc.BASE_FWD, sway_deg=4.0, sway_m=0.03,
                        gait_hz=0.9, noise_m=0.005, heading_bias_deg=0.0, latency=0.02, drift=0.0,
                        mc_expiry=1.0, pose_freeze_at=None, status_freeze_at=None, reject_windows=(),
                        jump_at=None, jump_dx=0.5, jump_dy=0.0, jump_deg=0.0, latency_at=None, latency_big=1.5,
                        nan_at=None, rc_off_at=None, rc_flick_at=None, rc_flick=(-0.2, 0.0, 0.0),
                        other_pnc_at=None, estop_at=None, mode_freeze_at=None, stop_at=None,
                        initial_mode=('STAND_DEFAULT', fc.RUNNING), status_text=None,
                        operator_driving_at_start=False, loc_walk=0.0, shadow_driver=None)
        unknown = set(kw) - set(self.cfg)
        assert not unknown, unknown
        self.cfg.update(kw)
        c = self.cfg
        self.mode_v = c['initial_mode']
        self.mode_t_change = 0.0
        self.pnc_cmds = []          # (t_sent, fwd, ang)
        self.rc_cmd = (0.0, 0.0, 0.0)
        self.last_nonzero_active = -1e9
        self.fwd_active, self.fwd_start = False, 0.0
        self.yaw_cmds = []          # (t, ang) of the active command, for the yaw lag
        self.poses, self.status, self.cmds = [], [], []
        self.sends = []             # what run_route sent: (t, fwd, ang)
        self.polled = None
        self.next_pose = self.next_status = self.next_rc = self.next_poll = 0.0
        self.icp_n = self.icp_ok = 0
        self.loc_bias = [0.0, 0.0]
        self.trace = []             # (t, x, y, yaw, v, w)

    # ---- IO interface used by run_route --------------------------------------------------
    def now(self):
        return self.t

    def stamp_now(self):
        return self.t

    def stop_requested(self):
        return self.cfg['stop_at'] is not None and self.t >= self.cfg['stop_at']

    def mode(self):
        return self.polled

    def send(self, fwd, ang):
        assert math.isfinite(fwd) and math.isfinite(ang)
        self.sends.append((self.t, fwd, ang))
        self.pnc_cmds.append((self.t, fwd, ang))
        self.cmds.append((self.t, fc.SOURCE, fwd, 0.0, ang))

    def poses_since(self, i):
        return self.poses[i:], len(self.poses)

    def status_since(self, i):
        return self.status[i:], len(self.status)

    def cmds_since(self, i):
        return self.cmds[i:], len(self.cmds)

    def sleep(self, dt):
        t_end = self.t + dt
        while self.t < t_end - 1e-9:
            self._step(min(0.01, t_end - self.t))

    # ---- physics -------------------------------------------------------------------------
    def _policy(self, cf, ca, t):
        """mc's walking policy as measured: (target forward speed, target yaw rate)"""
        c = self.cfg
        if cf >= (0.10 if self.fwd_active else 0.12):
            if not self.fwd_active:
                self.fwd_active, self.fwd_start = True, t
            v = c['creep_v'] if t - self.fwd_start < c['creep_s'] else min(c['v_cap'], max(c['v_walk'], 0.9 * cf))
        else:
            self.fwd_active = False
            v = 0.5 * cf if abs(cf) > 0.02 else 0.0
        a = abs(ca)
        if self.fwd_active:
            w = c['walk_turn_gain'] * ca
        elif a < 0.45:
            w = 0.0
        elif a < c['inplace_on']:
            w = math.copysign(0.11, ca)
        else:
            w = math.copysign(c['w_inplace'], ca)
        return v, w

    def _active_pnc(self, lag=None):
        """the pnc command mc is acting on: sent `lag` ago; expires mc_expiry after the last
        message (None: never expires -- only an explicit zero stops it)"""
        c = self.cfg
        t_eff = self.t - (c['lag'] if lag is None else lag)
        for (ts, f, a) in reversed(self.pnc_cmds):
            if ts <= t_eff:
                if c['mc_expiry'] is None or t_eff - ts <= c['mc_expiry']:
                    return (f, a)
                return (0.0, 0.0)
        return (0.0, 0.0)

    def _step(self, dt):
        c = self.cfg
        self.t += dt
        t = self.t
        rc_on = c['rc_off_at'] is None or t < c['rc_off_at']
        flick = c['rc_flick_at'] is not None and c['rc_flick_at'] <= t < c['rc_flick_at'] + 1.0
        drv = c['shadow_driver']
        if drv is not None:
            f, a = drv(self)
            self.rc_cmd = (f, 0.0, a)
        elif c['operator_driving_at_start'] and t < 1.0:
            self.rc_cmd = (0.3, 0.0, 0.0)
        else:
            self.rc_cmd = tuple(c['rc_flick']) if flick else (0.0, 0.0, 0.0)
        if rc_on and t >= self.next_rc:
            self.cmds.append((t, 'rc') + self.rc_cmd)
            self.next_rc = t + 0.02
            if c['other_pnc_at'] is not None and c['other_pnc_at'] <= t < c['other_pnc_at'] + 1.0:
                self.cmds.append((t, 'pnc-other', 0.1, 0.0, 0.0))   # another pnc publisher (adapter renames)
        if rc_on and any(self.rc_cmd):
            cf, ca = self.rc_cmd[0], self.rc_cmd[2]
            ca_y = ca
        else:
            cf, ca = self._active_pnc()
            ca_y = self._active_pnc(c['yaw_lag'])[1]          # turning reacts faster than walking
        nonzero = abs(cf) + abs(ca) > 1e-6
        desc, st = self.mode_v
        if c['estop_at'] is not None and t >= c['estop_at']:
            self.mode_v = ('SOFT_EMERGENCY_STOP', fc.RUNNING)
        elif desc == 'STAND_DEFAULT' and st == fc.RUNNING and nonzero and not c['never_walk']:
            self.mode_v = ('LOCOMOTION_DEFAULT', fc.TRANSITION); self.mode_t_change = t
        elif desc == 'LOCOMOTION_DEFAULT' and st == fc.TRANSITION and t - self.mode_t_change >= c['start_delay']:
            self.mode_v = ('LOCOMOTION_DEFAULT', fc.RUNNING)
        elif desc == 'LOCOMOTION_DEFAULT' and st == fc.RUNNING and t - self.last_nonzero_active > 3.6:
            self.mode_v = ('STAND_DEFAULT', fc.TRANSITION); self.mode_t_change = t
        elif desc == 'STAND_DEFAULT' and st == fc.TRANSITION and t - self.mode_t_change >= 1.5:
            self.mode_v = ('STAND_DEFAULT', fc.RUNNING)
        if nonzero:
            self.last_nonzero_active = t
        walking = self.mode_v == ('LOCOMOTION_DEFAULT', fc.RUNNING) and not c['stuck']
        if walking:
            tv, tw = self._policy(cf, ca_y, t)
            tv, tw = c['fwd_sign'] * tv, c['ang_sign'] * tw
        else:
            tv = tw = 0.0
            self.fwd_active = False
        self.v += min(1.0, dt / c['tau']) * (tv - self.v)
        self.w += min(1.0, dt / c['yaw_tau']) * (tw - self.w)
        self.yaw += self.w * dt
        push = c['drift'] if walking else 0.0
        self.x += self.v * math.cos(self.yaw) * dt - push * math.sin(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt + push * math.cos(self.yaw) * dt
        self.trace.append((t, self.x, self.y, self.yaw, self.v, self.w))
        if t >= self.next_poll and (c['mode_freeze_at'] is None or t < c['mode_freeze_at']):
            self.polled = (t, self.mode_v[0], self.mode_v[1])
            self.next_poll = t + 0.2
        if t >= self.next_pose and (c['pose_freeze_at'] is None or t < c['pose_freeze_at']):
            self.next_pose = t + 0.1
            moving = walking and (abs(self.v) > 0.02 or abs(self.w) > 0.02)
            ph = 2 * math.pi * c['gait_hz'] * t
            sway_h = math.radians(c['sway_deg']) * math.sin(ph) if moving else 0.0
            sway_l = c['sway_m'] * math.sin(ph) if moving else 0.0
            for k in (0, 1):
                self.loc_bias[k] = max(-0.15, min(0.15, self.loc_bias[k] + self.rng.gauss(0, c['loc_walk'])))
            cx = self.x + c['r_chest'] * math.cos(self.yaw) - sway_l * math.sin(self.yaw)
            cy = self.y + c['r_chest'] * math.sin(self.yaw) + sway_l * math.cos(self.yaw)
            cx += self.rng.gauss(0, c['noise_m']) + self.loc_bias[0]
            cy += self.rng.gauss(0, c['noise_m']) + self.loc_bias[1]
            h = self.yaw + sway_h + math.radians(c['heading_bias_deg'])
            if c['jump_at'] is not None and t >= c['jump_at']:
                cx += c['jump_dx']; cy += c['jump_dy']; h += math.radians(c['jump_deg'])
            lat = c['latency_big'] if (c['latency_at'] is not None and t >= c['latency_at']) else c['latency']
            q = quat_for_heading(h)
            if c['nan_at'] is not None and c['nan_at'] <= t < c['nan_at'] + 0.3:
                cx = float('nan')
            self.poses.append((t, t - lat, cx, cy, 0.0) + q)
        if t >= self.next_status and (c['status_freeze_at'] is None or t < c['status_freeze_at']):
            self.next_status = t + 1.0
            if c['status_text'] is not None:
                txt = c['status_text']
            else:
                self.icp_n += 1
                rej = any(a <= t < b for (a, b) in c['reject_windows'])
                self.icp_ok += 0 if rej else 1
                txt = (f'TRACKING frames {self.icp_n * 10}, ICP {self.icp_ok}/{self.icp_n} accepted, last '
                       f'{"REJECTED" if rej else "OK"} refine 0.0100 jump 0.010 m, queue 0, unpaired 0')
            self.status.append((t, txt))


def run(sim, route, P=None, align_final=False, shadow=False, verbose=False, crash_at=None, start=None):
    P = P or fc.Params()
    logs = []
    log = (lambda s: (logs.append(s), print('     ' + s))) if verbose else logs.append
    if crash_at is not None:              # a bug in the controller mid-leg
        orig = fc.Leg.step
        def bad(self, t, *a):
            if t >= crash_at:
                raise ValueError('simulated bug')
            return orig(self, t, *a)
        fc.Leg.step = bad
    try:
        res = fc.run_route(sim, P, route, align_final=align_final, shadow=shadow, log=log, start=start)
    except ValueError as e:
        res = {'outcome': 'EXCEPTION', 'reason': str(e), 'arrivals': []}
    finally:
        if crash_at is not None:
            fc.Leg.step = orig
    sim.sleep(4.0)          # no commands: the robot must come to rest
    res['logs'] = logs
    return res


def invariants(sim, res):
    bad = []
    for (t, f, a) in sim.sends:
        if not (0.0 <= f <= RULE_FWD + 1e-12 and abs(a) <= RULE_ANG + 1e-12):
            bad.append(f'cap violated at {t:.2f}: fwd {f} ang {a}')
            break
    if res['outcome'] == 'REFUSED':
        if sim.sends:
            bad.append(f'REFUSED but {len(sim.sends)} commands were sent')
    else:
        if not sim.sends:
            bad.append('no commands at all (expected at least the zero tail)')
        else:
            # the FIRST second after the last non-zero command must be continuous zeros
            # (a completed run then goes quiet while mc stands, and ends with another zero tail)
            nz = [i for i, s in enumerate(sim.sends) if s[1] or s[2]]
            t0 = sim.sends[nz[-1]][0] if nz else sim.sends[0][0]
            after = sim.sends[nz[-1] + 1:] if nz else sim.sends
            first = [s for s in after if s[0] <= t0 + 1.0]
            times = [t0] + [s[0] for s in first] if nz else [s[0] for s in first]
            gaps = [b - a for a, b in zip(times, times[1:])]
            span = times[-1] - times[0] if times else 0.0
            if len(first) < 15 or span < 0.95 or (gaps and max(gaps) > 0.1 + 1e-9):
                bad.append(f'zero tail: {len(first)} msgs over {span:.2f} s, max gap {max(gaps) if gaps else 0:.2f} s')
            if sim.sends[-1][1] or sim.sends[-1][2]:
                bad.append('the run ended on a non-zero command')
    end = sim.trace[-1]
    if abs(end[4]) > 0.01 or abs(end[5]) > 0.01:
        bad.append(f'robot still moving 4 s after the run: v {end[4]:.3f} w {end[5]:.3f}')
    return bad


SQUARE = [('a', 3.0, 0.0, 0.0), ('b', 3.0, 3.0, math.pi / 2), ('c', 0.0, 3.0, math.pi), ('home', 0.0, 0.0, 0.0)]
P0 = fc.Params()
# when must the last non-zero command have gone out, after a fault injected at T = 8 s?
T = 8.0
BY_POSE = T + P0.pose_stale_s + 0.15
BY_STATUS = T + P0.status_stale_s + 1.0 + 0.15      # status is 1 Hz: the last one may be up to 1 s old
BY_RC = T + P0.rc_stale_s + 0.15
BY_MODE = T + P0.mode_stale_s + 0.2 + 0.15          # poller is 5 Hz
BY_NOW = T + 0.3                                    # next tick(s) after the fault is visible


def scenarios():
    """(name, sim kwargs, route, expect outcome, reason substring, run kwargs, extra check)"""
    behind = [('behind', -2.0, 0.0, None)]            # exactly on the +-180 line
    one = [('a', 4.0, 0.0, None)]
    left = [('l', 0.0, 3.0, None)]
    home_start = ('home', 0.0, 0.0, 0.0)
    S = []
    S.append(('nominal square + align', {}, SQUARE, 'COMPLETED', None, dict(align_final=True), 'arrivals<0.35'))
    S.append(('target exactly behind (180 deg)', dict(noise_m=0.0), behind, 'COMPLETED', None, {}, 'flips<=1'))
    S.append(('slow start (2.6 s animation)', dict(start_delay=2.6), SQUARE[:2], 'COMPLETED', None, {}, 'arrivals<0.35'))
    S.append(('slow start 3.5 s + slow in-place turns', dict(start_delay=3.5, w_inplace=0.5), left, 'COMPLETED', None, {}, 'arrivals<0.35'))
    S.append(('fast in-place turns (1.5 rad/s)', dict(w_inplace=1.5), SQUARE, 'COMPLETED', None, {}, 'arrivals<0.35'))
    S.append(('walks slower (0.30 m/s)', dict(v_walk=0.30), SQUARE[:2], 'COMPLETED', None, {}, 'arrivals<0.35'))
    S.append(('walks faster (0.55 m/s)', dict(v_walk=0.55), SQUARE[:2], 'COMPLETED', None, {}, 'arrivals<0.35'))
    S.append(('C1: policy ignores the in-place turn', dict(w_inplace=0.0), left, 'ABORTED', 'no rotation', {}, None))
    S.append(('heading bias +5 deg', dict(heading_bias_deg=5.0), SQUARE, 'COMPLETED', None, {}, 'arrivals<0.35'))
    S.append(('heading bias -5 deg', dict(heading_bias_deg=-5.0), SQUARE, 'COMPLETED', None, {}, 'arrivals<0.35'))
    S.append(('chest offset really 0.09 m', dict(r_chest=0.09), SQUARE, 'COMPLETED', None, dict(align_final=True), 'arrivals<0.35'))
    S.append(('chest offset really 0.17 m', dict(r_chest=0.17), SQUARE, 'COMPLETED', None, dict(align_final=True), 'arrivals<0.35'))
    S.append(('7 ICP rejections in a row (loc_walk_01 max)', dict(reject_windows=((5.0, 12.0),)), one, 'COMPLETED', None, {}, 'arrivals<0.35'))
    S.append(('rejections 4-on/1-off, 32 in total', dict(reject_windows=tuple((5.0 + 5 * k, 9.0 + 5 * k) for k in range(8))),
              one, 'COMPLETED', None, {}, 'arrivals<0.35'))
    S.append(('pushed sideways (0.05 m/s)', dict(drift=0.05), one, 'COMPLETED', None, {}, None))
    S.append(('short leg + 0.22 m correction mid-turn', dict(jump_at=1.5, jump_dx=0.22), [('s', 0.0, 0.35, None)],
              'COMPLETED', None, {}, None))
    S.append(('start on the right waypoint', {}, one, 'COMPLETED', None, dict(start=home_start), None))
    S.append(('mc never starts walking', dict(never_walk=True), one, 'ABORTED', 'did not start walking', {},
              f'abort_by:{P0.start_wait_s + 0.3}'))
    S.append(('turn sign inverted', dict(ang_sign=-1.0), left, 'ABORTED', 'AGAINST the command', {}, 'turned<75'))
    S.append(('turn sign inverted, target behind', dict(ang_sign=-1.0), behind, 'ABORTED', 'AGAINST the command', {}, 'turned<75'))
    S.append(('forward sign inverted', dict(fwd_sign=-1.0), one, 'ABORTED', 'walking: getting worse', {}, 'moved<0.6'))
    S.append(('walking but not moving (stuck)', dict(stuck=True), left, 'ABORTED', 'no rotation', {}, None))
    S.append(('very slow robot (0.05 m/s)', dict(v_walk=0.05, creep_v=0.03, v_cap=0.05), [('far', 9.0, 0.0, None)], 'ABORTED', 'time limit', {}, None))
    S.append(('runs away (1.0 m/s)', dict(v_walk=1.0), [('far', 9.0, 0.0, None)], 'ABORTED', 'faster than', {}, None))
    S.append(('pose stream freezes', dict(pose_freeze_at=T), one, 'ABORTED', 'pose stale', {}, f'abort_by:{BY_POSE}'))
    S.append(('status stream freezes', dict(status_freeze_at=T), one, 'ABORTED', 'no localisation status', {}, f'abort_by:{BY_STATUS}'))
    S.append(('8 ICP rejections in a row', dict(reject_windows=((T - 7.5, 30.0),)), one, 'ABORTED', 'rejected in a row', {},
              f'abort_by:{T + 0.5 + 0.15}'))
    S.append(('pose jumps 0.5 m', dict(jump_at=T), one, 'ABORTED', 'pose jumped', {}, f'abort_by:{BY_NOW}'))
    S.append(('heading jumps 20 deg', dict(jump_at=T, jump_dx=0.0, jump_deg=20.0), one, 'ABORTED', 'pose jumped', {}, f'abort_by:{BY_NOW}'))
    S.append(('pose goes NaN', dict(nan_at=T), one, 'ABORTED', 'not finite', {}, f'abort_by:{BY_NOW}'))
    S.append(('pose 1.5 s late', dict(latency_at=T), one, 'ABORTED', 'behind real time', {}, f'abort_by:{BY_NOW}'))
    S.append(('operator flicks forward', dict(rc_flick_at=T), one, 'ABORTED', 'operator took over', {}, f'abort_by:{BY_NOW}'))
    S.append(('operator flicks turn only', dict(rc_flick_at=T, rc_flick=(0.0, 0.0, 0.5)), one, 'ABORTED', 'operator took over', {}, f'abort_by:{BY_NOW}'))
    S.append(('operator flicks sideways only', dict(rc_flick_at=T, rc_flick=(0.0, 0.3, 0.0)), one, 'ABORTED', 'operator took over', {}, f'abort_by:{BY_NOW}'))
    S.append(('operator nudges 0.02', dict(rc_flick_at=T, rc_flick=(0.02, 0.0, 0.0)), one, 'ABORTED', 'operator took over', {}, f'abort_by:{BY_NOW}'))
    S.append(('another pnc publisher', dict(other_pnc_at=T), one, 'ABORTED', 'another command source', {}, f'abort_by:{BY_NOW}'))
    S.append(('controller disconnects', dict(rc_off_at=T), one, 'ABORTED', 'remote controller', {}, f'abort_by:{BY_RC}'))
    S.append(('remote e-stop', dict(estop_at=T), one, 'ABORTED', 'motion mode', {}, f'abort_by:{T + 0.2 + 0.15}'))
    S.append(('mode service stops answering', dict(mode_freeze_at=T), one, 'ABORTED', 'motion mode unknown', {}, f'abort_by:{BY_MODE}'))
    S.append(('pushed sideways hard (0.12 m/s)', dict(drift=0.12), [('far', 8.0, 0.0, None)], 'ABORTED', 'off the straight line', {}, None))
    S.append(('Ctrl-C mid-leg', dict(stop_at=T), one, 'ABORTED', 'stop requested', {}, f'abort_by:{BY_NOW}'))
    S.append(('bug in the controller mid-leg', {}, one, 'EXCEPTION', 'simulated bug', dict(crash_at=T), f'abort_by:{BY_NOW}'))
    S.append(('refuse: not on the start waypoint', dict(x=0.5), one, 'REFUSED', 'not on the start waypoint', dict(start=home_start), None))
    S.append(('refuse: start heading off by 30 deg', dict(yaw=math.radians(30)), one, 'REFUSED', 'not on the start waypoint', dict(start=home_start), None))
    S.append(('refuse: leg > 10 m', {}, [('far', 12.0, 0.0, None)], 'REFUSED', '> 10.0 m', {}, None))
    S.append(('refuse: 2nd leg > 10 m', {}, [('a', 3.0, 0.0, None), ('far', 3.0, 11.0, None)], 'REFUSED', '> 10.0 m', {}, None))
    S.append(('refuse: localiser still searching', dict(status_text='SEARCHING: collected 10/30 startup scans'), one, 'REFUSED', 'not tracking', {}, None))
    S.append(('refuse: robot in damping', dict(initial_mode=('DAMPING_DEFAULT', fc.RUNNING)), one, 'REFUSED', 'motion mode', {}, None))
    S.append(('refuse: robot mid-transition', dict(initial_mode=('STAND_DEFAULT', fc.TRANSITION)), one, 'REFUSED', 'motion mode', {}, None))
    S.append(('refuse: controller off', dict(rc_off_at=0.0), one, 'REFUSED', 'remote controller', {}, None))
    S.append(('refuse: operator driving', dict(operator_driving_at_start=True), one, 'REFUSED', 'operator took over', {}, None))
    S.append(('refuse: no localisation', dict(pose_freeze_at=0.0), one, 'REFUSED', 'no localisation pose', {}, None))
    S.append(('refuse: NaN pose', dict(nan_at=0.0), one, 'REFUSED', 'not finite', {}, None))
    return S


def check_extra(kind, sim, res):
    if kind is None:
        return []
    if kind == 'arrivals<0.35':
        return [f"arrival at {a['name']} {a['err_m']:.2f} m" for a in res['arrivals'] if a['err_m'] > 0.35]
    if kind == 'turned<75':     # in place at ~1 rad/s: detection (25 deg) + filter lag + carry-on
        y0 = sim.trace[0][3]
        worst = max(abs(math.degrees(fc.wrap(tr[3] - y0))) for tr in sim.trace)
        return [] if worst < 75 else [f'turned {worst:.0f} deg the wrong way before stopping']
    if kind == 'moved<0.6':
        x0, y0 = sim.trace[0][1], sim.trace[0][2]
        worst = max(math.hypot(tr[1] - x0, tr[2] - y0) for tr in sim.trace)
        return [] if worst < 0.6 else [f'moved {worst:.2f} m the wrong way before stopping']
    if kind == 'flips<=1':
        signs = [1 if a > 0 else -1 for (_, f, a) in sim.sends if a != 0.0 and f == 0.0]
        flips = sum(1 for p, q in zip(signs, signs[1:]) if p != q)
        return [] if flips <= 1 else [f'turn direction changed {flips} times']
    if kind.startswith('abort_by:'):
        lim = float(kind.split(':')[1])
        nz = [s for s in sim.sends if s[1] or s[2]]
        if not nz:
            return []
        return [] if nz[-1][0] <= lim else [f'last non-zero command at {nz[-1][0]:.2f} s > {lim:.2f} s']
    return [f'unknown check {kind}']


def shadow_test(verbose):
    """Operator drives (scripted pursuit on the rc channel). The core still calls io.send (the
    ROS adapter has no publisher in shadow; test_follower_ros.py checks nothing goes out);
    here: operator commands do not trip, and arrivals are measured."""
    route = [('a', 3.0, 0.0, None), ('b', 3.0, 2.0, None)]
    goals = iter(route)
    state = {'g': next(goals)}

    def driver(sim):
        g = state['g']
        if g is None:
            return (0.0, 0.0)
        dx, dy = g[1] - sim.x, g[2] - sim.y
        if math.hypot(dx, dy) < 0.15:
            state['g'] = next(goals, None)
            return (0.0, 0.0)
        err = fc.wrap(math.atan2(dy, dx) - sim.yaw)
        if abs(err) > 0.3:
            return (0.0, math.copysign(0.7, err))          # in place: above mc's ~0.5 threshold
        return (0.3, max(-0.5, min(0.5, 1.2 * err)))

    sim = Sim(seed=3, shadow_driver=driver)
    res = run(sim, route, shadow=True, verbose=verbose)
    errs = []
    if res['outcome'] != 'COMPLETED':
        errs.append(f"outcome {res['outcome']} {res.get('reason')}")
    if len(res['arrivals']) != 2:
        errs.append(f"{len(res['arrivals'])} arrivals measured, expected 2")
    if any('operator took over' in w for w in res.get('would_abort', [])):
        errs.append('operator commands tripped the shadow run')
    return errs, res


def random_routes(n, verbose):
    errs, all_err = [], []
    for k in range(n):
        rng = random.Random(1000 + k)
        x = y = 0.0
        route = []
        for i in range(rng.randint(1, 4)):
            d, b = rng.uniform(0.5, 8.0), rng.uniform(-math.pi, math.pi)
            x, y = x + d * math.cos(b), y + d * math.sin(b)
            route.append((f'w{i}', x, y, rng.uniform(-math.pi, math.pi)))
        sim = Sim(seed=k, yaw=rng.uniform(-math.pi, math.pi), start_delay=rng.choice([0.23, 0.5, 1.5, 2.6]),
                  heading_bias_deg=rng.uniform(-5, 5), lag=rng.uniform(0.2, 0.5), tau=rng.uniform(0.25, 0.5),
                  v_walk=rng.uniform(0.40, 0.52), creep_s=rng.uniform(2.0, 3.5), w_inplace=rng.uniform(0.5, 1.2),
                  walk_turn_gain=rng.uniform(0.8, 1.3), yaw_lag=rng.uniform(0.15, 0.35), yaw_tau=rng.uniform(0.08, 0.2),
                  loc_walk=0.003, r_chest=rng.uniform(0.09, 0.17))
        res = run(sim, route, align_final=rng.random() < 0.5)
        bad = invariants(sim, res)
        if res['outcome'] != 'COMPLETED':
            bad.append(f"outcome {res['outcome']}: {res.get('reason')}")
        for a in res['arrivals']:
            all_err.append(a['err_m'])
            if a['err_m'] > 0.35:
                bad.append(f"arrival {a['name']} {a['err_m']:.2f} m")
        true_end = math.hypot(sim.x - route[-1][1], sim.y - route[-1][2])   # sim truth, not the estimate
        if true_end > 0.45:
            bad.append(f'TRUE end position {true_end:.2f} m from the last waypoint')
        if bad:
            errs.append((k, bad, res['logs'][-6:]))
    all_err.sort()
    return errs, all_err


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('-v', '--verbose', action='store_true')
    ap.add_argument('--random', type=int, default=150, help='number of random routes')
    ap.add_argument('-k', default=None, help='only scenarios whose name contains this')
    a = ap.parse_args()
    # the rule's caps, literally (a "tuned" constant must fail here, not pass quietly)
    assert (fc.MAX_FWD, fc.MAX_ANG, fc.MAX_LEG) == (RULE_FWD, RULE_ANG, RULE_LEG), (fc.MAX_FWD, fc.MAX_ANG, fc.MAX_LEG)
    P0_ = fc.Params()
    assert P0_.fwd_cmd <= RULE_FWD and P0_.ang_turn <= RULE_ANG and P0_.ang_walk_max <= 0.30
    # the follower's robot-forward must match the measured one (tools/heading_check.py on recorded walks)
    dot = sum(p * q for p, q in zip(fc.F_BODY, F_TRUE)) / math.sqrt(sum(p * p for p in fc.F_BODY) * sum(q * q for q in F_TRUE))
    assert dot > math.cos(math.radians(6)), f'fc.F_BODY is {math.degrees(math.acos(min(1, dot))):.0f} deg off the measured forward'
    for deg in range(-180, 181, 15):
        q = quat_for_heading(math.radians(deg))
        got = math.degrees(fc.heading_from_quat(*q))
        assert abs(fc.wrap(math.radians(got - deg))) < 0.05, (deg, got)
    print('ok  caps are the rule\'s literal values; F_BODY matches the measured forward; heading round-trips')
    # re-turn livelock (review): the estimate lands past the goal's perpendicular at > near_m
    # while the robot faces the goal. It must walk in and arrive, not cycle TURN/WALK into an abort.
    leg = fc.Leg(P0, 'g', (0.0, 0.0), (4.0, 0.0), t0=0.0)
    leg.state = 'WALK'
    x, y, t = 4.25, 0.45, 0.0                  # along the original line: -0.25 m; dist 0.51 m
    for _ in range(400):
        h = math.atan2(0.0 - y, 4.0 - x)       # facing the goal
        fwd, ang, ev = leg.step(t, x, y, h, True)
        if leg.state == 'ARRIVED':
            break
        assert leg.state == 'WALK', (leg.state, ev)
        x += fwd * math.cos(h) * 0.05; y += fwd * math.sin(h) * 0.05; t += 0.05
    assert leg.state == 'ARRIVED' and leg.returns == 0, (leg.state, leg.returns)
    print('ok  estimate past the goal line, facing the goal: walks in, no re-turn loop')
    fails = 0
    for expiry in (1.0, None):
        tag_e = '' if expiry else ' [no mc expiry]'
        for (name, kw, route, want, why, extra, chk) in scenarios():
            if a.k and a.k not in name:
                continue
            kw = dict(kw, mc_expiry=expiry)
            sim = Sim(seed=1, **kw)
            if a.verbose:
                print(f'--- {name}{tag_e}')
            res = run(sim, route, verbose=a.verbose, **extra)
            bad = invariants(sim, res)
            if res['outcome'] != want:
                bad.append(f"outcome {res['outcome']} (want {want}): {res.get('reason')}")
            if why and why not in (res.get('reason') or ''):
                bad.append(f"reason {res.get('reason')!r} lacks {why!r}")
            bad += check_extra(chk, sim, res)
            ok = not bad
            fails += not ok
            if expiry or not ok:
                detail = res.get('reason') or ', '.join(f"{x['name']} {x['err_m']:.2f} m" for x in res['arrivals'])
                print(f'{"ok  " if ok else "FAIL"} {name + tag_e:<46} {res["outcome"]:<9} {detail[:110]}')
                for b in bad:
                    print(f'       !! {b}')
        if not a.k:
            print(f'ok  (scenario list{tag_e or " with mc expiry"} done)')
    if not a.k or 'shadow' in a.k:
        bad, res = shadow_test(a.verbose)
        fails += bool(bad)
        print(f"{'ok  ' if not bad else 'FAIL'} {'shadow: operator drives, follower observes':<46} {res['outcome']:<9} "
              f"{', '.join(f'''{x['name']} {x['err_m']:.2f} m''' for x in res['arrivals'])}")
        for b in bad:
            print(f'       !! {b}')
    if a.random and not a.k:
        errs, all_err = random_routes(a.random, a.verbose)
        fails += len(errs)
        n = len(all_err)
        print(f"{'ok  ' if not errs else 'FAIL'} {a.random} random routes ({n} arrivals): arrival error median "
              f"{all_err[n // 2]:.3f} m, p95 {all_err[int(0.95 * n)]:.3f} m, max {all_err[-1]:.3f} m")
        for k, bad, tail in errs[:10]:
            print(f'       !! route {k}: {bad}')
            for line in tail:
                print(f'            {line}')
    print(f'\n{"ALL PASSED" if not fails else f"{fails} FAILED"}')
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
