"""Waypoint follower core: pure Python, no ROS, so it runs unchanged against the simulator
(tools/test_follower.py) and on the robot (tools/waypoint_follower.py).

The robot walks straight legs: from where it stands to the next waypoint. Per leg:
  TURN   turn in place at a FIXED ang_turn until within max(turn_stop_min_deg, measured
         rate x turn_lead_s) of the bearing, then let it coast (mc's walking policy ignores in-place turn commands below ~0.5 rad/s,
         and above that turns at ~0.5-1.0 rad/s: README "Stage C1")
  WALK   ONE forward command, fwd_cmd (mc walks at ~0.45 m/s for any command 0.12-0.2;
         below 0.1 it stops), steering toward the goal with <= ang_walk_max (turning WHILE
         walking does track small commands); > return_deg off -> TURN again (and the approach
         line is rebuilt from where the robot is)
  arrive forward stops at arrive_trigger_m (the robot coasts ~0.2-0.3 m), or on passing
         abeam of the goal inside near_m -> zeros, wait for stand
  ALIGN  (optional, last waypoint) turn in place toward the waypoint's saved heading

Frames and signs (README "Steering facts for the waypoint follower"):
  - map frame is z-up; heading = atan2(y, x) of R_map_body @ F_BODY, projected horizontal
  - angular_velocity > 0 turns the robot counter-clockwise (left) in the map
  - F_BODY (robot-forward in the IMU frame) varies ~5 deg between walks: heading is +-3 deg
  - positions are the TURNING CENTRE (BASE_FWD behind the chest IMU), not the chest

Every exit after preflight ends in zeros, sent BEFORE anything is printed. run_route() is the
only place commands are produced; the IO object it drives clamps them again before publishing.
"""
import math
import re
from dataclasses import dataclass

# ---- HARD CAPS on the COMMAND. Enforced in run_route.send and again in every IO.send(). ------
# They bound what is sent, not the speed: mc walks at ~0.45 m/s for any forward command of
# 0.12-0.2, and turns in place at ~0.5-1.0 rad/s for 0.5-0.6 (README "Stage C1").
MAX_FWD = 0.20          # m/s commanded; forward only, never backwards
MAX_ANG = 0.60          # rad/s commanded; +left. 0.6 is for turning IN PLACE (operator, 2026-10-21)
MAX_LEG = 10.0          # m; a longer leg is refused
SOURCE = 'pnc'          # registered planning-and-control source, priority 40 < rc 80

# robot-forward in the IMU (body) frame: normalised mean over six walks
# (map_walk_01, test_walk_01, loc_walk_01, live_walk_01, marked_loop_02/03)
F_BODY = (0.0158, -0.0213, -0.9992)

# The pose is the chest IMU's, which is ahead of the point the robot turns about. Fitting
# chest = centre + Rz(heading) @ offset over slow in-place turns (map_walk_01 x3,
# marked_loop_02 x1, fit rms 1-2 cm) gives 0.11-0.15 m forward, ~0 sideways (static tf:
# IMU 0.09 m ahead of torso_link). Everything steers and measures the TURNING CENTRE
# ("base point") = chest - BASE_FWD * forward, so turning in place does not move it.
BASE_FWD = 0.13

START_ACTIONS = ('STAND_DEFAULT', 'LOCOMOTION_DEFAULT')
RUNNING, TRANSITION = 100, 200


@dataclass
class Params:
    # measured (follow_c_01): in place, |ang| <= 0.45 -> no turn, 0.50-0.55 -> 0.48 rad/s,
    # >= 0.55 -> ~0.95; walking, ang 0.2-0.3 -> ~0.27 rad/s; forward 0.10-0.13 -> 2.5-3 s of
    # creeping at 0.02-0.09 m/s, then 0.42-0.51 m/s
    fwd_cmd: float = 0.15        # the ONE forward command while walking (>= 0.12 so mc walks)
    ang_turn: float = 0.60       # in-place turn command, fixed (the policy needs > ~0.5)
    # the in-place command stops when the remaining angle < max(turn_stop_min_deg, rate x
    # turn_lead_s): after release the robot carries on ~0.25-0.4 s (11 deg median, 28 max over
    # 6 operator releases at 0.4-1.7 rad/s), plus ~0.3 s of heading-filter lag
    turn_stop_min_deg: float = 15.0
    turn_lead_s: float = 0.70       # errs toward stopping SHORT: walking steering finishes the turn,
    turn_rate_prior: float = 0.6    # while an overshoot past return_deg would re-turn (oscillate)
    walk_settle_s: float = 1.5      # no re-turn this soon after a turn: its carry-on is still settling
    ang_walk_max: float = 0.30   # steering cap while walking
    k_head: float = 0.6          # WALK: ang = k_head * err
    return_deg: float = 30.0     # WALK -> TURN beyond this (unless inside near_m)
    arrive_trigger_m: float = 0.35   # forward stops here; ~0.45 m/s x (lag + decel) coasts in
    near_m: float = 0.50         # inside: no steering; passing abeam counts as arrived
    walk_speed_min: float = 0.20     # for the leg time limit only (conservative)
    max_speed: float = 0.80      # m/s over 1 s of poses: faster than mc should ever walk us
    heading_window_s: float = 0.6   # circular mean; gait sway is p50 3.4, p95 10 deg
    # localisation gate (loc_walk_01: 10.0 Hz, no gap > 0.1 s; max 7 consecutive ICP
    # rejections, in the height-swing window; accepted jumps max 0.22 m)
    pose_stale_s: float = 0.5
    pose_latency_s: float = 1.0
    status_stale_s: float = 3.0
    max_rejects: int = 8
    max_pose_step_m: float = 0.4
    max_pose_step_deg: float = 15.0
    # motion mode, operator
    mode_stale_s: float = 1.0
    rc_stale_s: float = 1.0      # the remote streams ~49 Hz while connected, zeros included
    start_wait_s: float = 4.0    # stand -> walk took 0.23 s, and 2.6 s behind an animation
    # the start: the operator puts the robot on a waypoint; localisation must agree
    start_tol_m: float = 0.30
    start_tol_deg: float = 15.0
    # wrong-way / progress watchdogs (armed only once mc reports walking)
    worse_deg: float = 25.0      # TURN/ALIGN: rotated this far AGAINST the command
    worse_m: float = 0.30        # WALK: distance grew this much past its best
    stall_s: float = 6.0
    stall_deg: float = 5.0
    stall_m: float = 0.10
    max_flips: int = 4           # TURN/ALIGN: direction reversals (oscillating about the target)
    fence_m: float = 1.0         # max sideways distance from the leg's ORIGINAL straight line
    max_returns: int = 3
    leg_time_margin_s: float = 30.0
    # loop
    preflight_wait_s: float = 8.0   # DDS discovery of a fresh process; returns as soon as all inputs arrived
    rate_hz: float = 20.0
    zero_tail_s: float = 1.0
    stand_wait_s: float = 6.0
    settle_s: float = 1.0


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_to_rot(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return ((1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
            (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
            (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)))


def heading_from_quat(qx, qy, qz, qw, f=F_BODY):
    """Robot heading in the map: R_map_body @ f, projected horizontal. Valid at any mounting
    (no Euler angles: the IMU is pitched ~-85 deg, gimbal lock for yaw/pitch/roll)."""
    R = quat_to_rot(qx, qy, qz, qw)
    fx = R[0][0] * f[0] + R[0][1] * f[1] + R[0][2] * f[2]
    fy = R[1][0] * f[0] + R[1][1] * f[1] + R[1][2] * f[2]
    return math.atan2(fy, fx)


def base_point(x, y, heading):
    """turning centre from the chest position and the robot heading"""
    return x - BASE_FWD * math.cos(heading), y - BASE_FWD * math.sin(heading)


def circ_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class Abort(Exception):
    pass


class Leg:
    """One leg, as a state machine. step() returns (fwd, ang, event); raises Abort."""

    def __init__(self, P, name, start_xy, goal_xy, goal_heading=None, t0=0.0):
        self.P, self.name = P, name
        self.gx, self.gy = goal_xy
        self.goal_heading = goal_heading           # radians, for ALIGN; None = no align
        self.length = math.hypot(self.gx - start_xy[0], self.gy - start_xy[1])
        if self.length > MAX_LEG:
            raise Abort(f'leg to {name} is {self.length:.1f} m > {MAX_LEG} m limit')
        self._approach(*start_xy)
        # the fence is measured from the ORIGINAL line and never moves
        self.fx, self.fy, self.fux, self.fuy = self.sx, self.sy, self.ux, self.uy
        self.state = 'TURN' if self.length >= P.arrive_trigger_m else ('ALIGN' if goal_heading is not None else 'ARRIVED')
        self.t0 = t0
        self.time_limit = self.length / P.walk_speed_min + P.leg_time_margin_s + (
            2 * math.pi / 0.5 if goal_heading is not None else 0.0)
        self.returns = 0
        self.turn_dir = 0.0
        self.flips = 0
        self._reset_watch(t0, None)
        self.rot_h = None
        self.hist = []                             # (t, unwrapped heading) for the turn rate
        self.t_turned = -1e9

    def _approach(self, x, y):
        """(re)build the approach line from (x, y) to the goal: `along` and the passed-abeam
        arrival refer to it"""
        self.sx, self.sy = x, y
        dx, dy = self.gx - x, self.gy - y
        n = math.hypot(dx, dy)
        self.ux, self.uy = (dx / n, dy / n) if n > 1e-6 else (1.0, 0.0)

    def _reset_watch(self, t, metric):
        self.best, self.best_t = metric, t

    def _watch(self, t, metric, moving, unit_worse, unit_stall, what):
        """WALK progress watchdog on the distance (must DEcrease); armed only while moving"""
        if not moving or self.best is None:
            self._reset_watch(t, metric)
            return
        if metric < self.best - unit_stall:
            self._reset_watch(t, metric)
        elif metric > self.best + unit_worse:
            raise Abort(f'{what}: getting worse ({metric:.2f} vs best {self.best:.2f}) -- wrong sign or pushed off')
        elif t - self.best_t > self.P.stall_s:
            raise Abort(f'{what}: no progress for {self.P.stall_s:.0f} s (best {self.best:.2f}) -- stuck?')

    def _rot_watch(self, t, h, moving, what):
        """TURN/ALIGN watchdog on the robot's OWN rotation in the commanded direction. Position
        corrections (ICP jumps up to 0.22 m) move the bearing to the goal but not this."""
        P = self.P
        if not moving or self.rot_h is None:
            self.rot_h, self.rot_acc, self.rot_best, self.rot_best_t = h, 0.0, 0.0, t
            return
        self.rot_acc += wrap(h - self.rot_h)
        self.rot_h = h
        prog = math.degrees(self.turn_dir * self.rot_acc)
        if prog < -P.worse_deg:
            raise Abort(f'{what}: rotated {-prog:.0f} deg AGAINST the command -- turn sign wrong?')
        if prog > self.rot_best + P.stall_deg:
            self.rot_best, self.rot_best_t = prog, t
        elif t - self.rot_best_t > P.stall_s:
            raise Abort(f'{what}: no rotation for {P.stall_s:.0f} s -- stuck?')

    def geometry(self, x, y):
        ex, ey = self.gx - x, self.gy - y
        dist = math.hypot(ex, ey)
        along = ex * self.ux + ey * self.uy                          # remaining, on the approach line
        cross = (x - self.fx) * self.fuy - (y - self.fy) * self.fux  # sideways off the ORIGINAL line
        return dist, math.atan2(ey, ex), along, cross

    def _rate(self, t, h):
        """the robot's own turn rate (rad/s) over the last ~0.4 s of filtered headings"""
        if self.hist:
            h = self.hist[-1][1] + wrap(h - self.hist[-1][1])
        self.hist.append((t, h))
        self.hist = [p for p in self.hist if t - p[0] <= 0.4]
        (t0, h0), (t1, h1) = self.hist[0], self.hist[-1]
        return (h1 - h0) / (t1 - t0) if t1 - t0 >= 0.2 else 0.0

    def _turn_stop(self, t, heading):
        rate = max(abs(self._rate(t, heading)), self.P.turn_rate_prior)      # short turns: still accelerating
        return max(math.radians(self.P.turn_stop_min_deg), rate * self.P.turn_lead_s)

    def _turn_cmd(self, err):
        """fixed-rate in-place turn (bang-bang): mc's policy ignores small in-place commands"""
        P = self.P
        # hysteresis near +-180 deg: keep the chosen direction, so noise cannot flip it
        if abs(err) < math.radians(150) or self.turn_dir == 0.0:
            d = 1.0 if err >= 0 else -1.0
            if self.turn_dir != 0.0 and d != self.turn_dir:
                self.flips += 1
                if self.flips > P.max_flips:
                    raise Abort(f'leg to {self.name}: turn reversed {self.flips} times (oscillating)')
                self.rot_h = None                     # re-arm the rotation watch for the new direction
            self.turn_dir = d
        return self.turn_dir * P.ang_turn

    def _enter_turn(self, t):
        self.state = 'TURN'
        self.turn_dir = 0.0
        self.rot_h = None
        self._reset_watch(t, None)

    def step(self, t, x, y, heading, moving):
        P = self.P
        if t - self.t0 > self.time_limit:
            raise Abort(f'leg to {self.name}: time limit {self.time_limit:.0f} s exceeded')
        dist, bearing, along, cross = self.geometry(x, y)
        if abs(cross) > P.fence_m and self.state in ('TURN', 'WALK'):
            raise Abort(f'{abs(cross):.2f} m off the straight line to {self.name} (fence {P.fence_m} m)')
        if self.state == 'TURN':
            err = wrap(bearing - heading)
            if abs(err) < self._turn_stop(t, heading):
                self.state = 'WALK'
                self.hist = []
                self.t_turned = t
                self._reset_watch(t, None)
                return 0.0, 0.0, 'turned'
            ang = self._turn_cmd(err)
            self._rot_watch(t, heading, moving, 'turning')
            return 0.0, ang, None
        if self.state == 'WALK':
            if dist < P.arrive_trigger_m or (dist < P.near_m and along <= 0.0):
                self.state = 'ALIGN' if self.goal_heading is not None else 'ARRIVED'
                self.turn_dir = 0.0
                self.rot_h = None
                self.flips = 0
                self.hist = []
                return 0.0, 0.0, 'arrived' if dist < P.arrive_trigger_m else 'arrived (passed abeam)'
            err = wrap(bearing - heading)
            if dist >= P.near_m and abs(err) > math.radians(P.return_deg) and t - self.t_turned > P.walk_settle_s:
                self.returns += 1
                if self.returns > P.max_returns:
                    raise Abort(f'leg to {self.name}: re-turned {self.returns} times')
                self._approach(x, y)
                self._enter_turn(t)
                self.hist = []
                return 0.0, 0.0, f're-turn ({math.degrees(err):+.0f} deg off)'
            self._watch(t, dist, moving, P.worse_m, P.stall_m, 'walking')
            ang = 0.0 if dist < P.near_m else clamp(P.k_head * err, -P.ang_walk_max, P.ang_walk_max)
            return P.fwd_cmd, ang, None
        if self.state == 'ALIGN':
            err = wrap(self.goal_heading - heading)
            if abs(err) < self._turn_stop(t, heading):
                self.state = 'ARRIVED'
                return 0.0, 0.0, 'aligned'
            ang = self._turn_cmd(err)
            self._rot_watch(t, heading, moving, 'aligning')
            return 0.0, ang, None
        return 0.0, 0.0, None


_TRACK = re.compile(r'^TRACKING .* last (OK|REJECTED)')


class Gate:
    """Everything that must hold before and while commanding: localisation, motion mode, the
    operator's controller. Consumes the IO's append-only message lists by index."""

    def __init__(self, P, shadow=False):
        self.P, self.shadow = P, shadow
        self.pi = self.si = self.ci = 0
        self.poses = []                 # (t_recv, stamp, base x, base y, z, heading) recent window
        self.last_pose = None
        self.status_t, self.status = None, None
        self.rejects = 0
        self.rc_t = None
        self.rc_cmd = (0.0, 0.0, 0.0)   # latest remote command (fwd, lat, ang), for the logs
        self.foreign = None             # (t, source, fwd, lat, ang) of the last non-zero non-pnc command
        self.pose_fault = None
        self.nonfinite = False
        self.track = []                 # (stamp, base x, base y), last ~1.5 s, for the speed check

    def update(self, io):
        P = self.P
        new, self.pi = io.poses_since(self.pi)
        for (tr, st, cx, cy, z, qx, qy, qz, qw) in new:
            vals = (st, cx, cy, z, qx, qy, qz, qw)
            if not all(math.isfinite(v) for v in vals) or qx * qx + qy * qy + qz * qz + qw * qw < 0.5:
                self.nonfinite = True   # sticky; the jump reference stays the last finite pose
                continue
            h = heading_from_quat(qx, qy, qz, qw)
            x, y = base_point(cx, cy, h)
            if self.last_pose is not None:
                _, _, lx, ly, _, lh = self.last_pose
                step = math.hypot(x - lx, y - ly)
                dh = abs(math.degrees(wrap(h - lh)))
                if step > P.max_pose_step_m or dh > P.max_pose_step_deg:
                    self.pose_fault = f'pose jumped {step:.2f} m / {dh:.1f} deg in one update'
            self.last_pose = (tr, st, x, y, z, h)
            self.poses.append(self.last_pose)
            self.track.append((st, x, y))
        if self.poses:
            st_last = self.poses[-1][1]
            self.poses = [p for p in self.poses if st_last - p[1] <= P.heading_window_s]
            self.track = [p for p in self.track if st_last - p[0] <= 1.5]
        new, self.si = io.status_since(self.si)
        for (tr, text) in new:
            self.status_t, self.status = tr, text
            m = _TRACK.match(text)
            if m:
                self.rejects = 0 if m.group(1) == 'OK' else self.rejects + 1
        new, self.ci = io.cmds_since(self.ci)
        for (tr, src, fwd, lat, ang) in new:
            if src == 'rc':
                self.rc_t = tr
                self.rc_cmd = (fwd, lat, ang)
            if src != SOURCE and abs(fwd) + abs(lat) + abs(ang) > 1e-3:
                self.foreign = (tr, src, fwd, lat, ang)

    def speed(self):
        """horizontal speed of the turning centre over the last >= 1 s of poses (None if < 1 s)"""
        if len(self.track) < 2 or self.track[-1][0] - self.track[0][0] < 1.0:
            return None
        (t0, x0, y0), (t1, x1, y1) = self.track[0], self.track[-1]
        return math.hypot(x1 - x0, y1 - y0) / (t1 - t0)

    def pose(self):
        """(x, y, filtered heading): turning-centre position from the latest pose, heading
        averaged over heading_window_s (gait sway). None if no pose yet."""
        if not self.poses:
            return None
        _, _, x, y, _, _ = self.poses[-1]
        return x, y, circ_mean([p[5] for p in self.poses])

    def faults(self, io, since_t, need_running=False):
        """list of reasons it is not safe to command right now (empty = OK).
        since_t: other sources' commands after this time count as a takeover."""
        P, now, out = self.P, io.now(), []
        if self.nonfinite:
            out.append('localisation pose not finite (NaN/inf)')
        if self.last_pose is None:
            out.append('no localisation pose received')
        else:
            tr, st = self.last_pose[0], self.last_pose[1]
            if now - tr > P.pose_stale_s:
                out.append(f'localisation pose stale ({now - tr:.2f} s since last)')
            lat = io.stamp_now() - st
            if lat > P.pose_latency_s:
                out.append(f'localisation pose {lat:.2f} s behind real time')
        v = self.speed()
        if v is not None and v > P.max_speed:
            out.append(f'moving at {v:.2f} m/s, faster than the {P.max_speed} m/s limit')
        if self.pose_fault:                 # one-shot: reported once, then cleared
            out.append(self.pose_fault)
            self.pose_fault = None
        if self.status_t is None or now - self.status_t > P.status_stale_s:
            out.append('no localisation status in the last %.0f s' % P.status_stale_s)
        elif not self.status.startswith('TRACKING'):
            out.append(f'localiser not tracking: {self.status[:60]}')
        if self.rejects >= P.max_rejects:
            out.append(f'{self.rejects} ICP matches rejected in a row')
        md = io.mode()
        if md is None or now - md[0] > P.mode_stale_s:
            out.append('motion mode unknown (GetMcAction not answering)')
        else:
            _, desc, status = md
            allowed = (RUNNING,) if need_running else (RUNNING, TRANSITION)
            if desc not in START_ACTIONS or status not in allowed:
                out.append(f'motion mode {desc} status={status}')
        if self.rc_t is None or now - self.rc_t > P.rc_stale_s:
            out.append('no signal from the remote controller (must be on and connected)')
        if not self.shadow and self.foreign is not None and self.foreign[0] >= since_t:
            _, src, f, l, a = self.foreign
            who = 'operator took over' if src == 'rc' else 'another command source is active'
            out.append(f'{who} ({src}: fwd {f:+.2f} lat {l:+.2f} ang {a:+.2f})')
        return out

    def walking(self, io):
        md = io.mode()
        return md is not None and md[1] == 'LOCOMOTION_DEFAULT' and md[2] == RUNNING


def preflight(io, gate, route, shadow=False, wait_s=2.0, start=None, P=None):
    """(faults, (x, y, heading) or None). Reads only; publishes nothing.
    start: (name, x, y, heading_rad) of the waypoint the operator put the robot on; the
    localised pose must agree with it, or a wrong-but-confident localisation could steer."""
    P = P or gate.P
    t_wait = io.now()
    while io.now() - t_wait < wait_s:          # let subscriptions and the mode poller fill
        gate.update(io)
        if gate.pose() is not None and gate.status_t is not None and gate.rc_t is not None and io.mode():
            break
        io.sleep(0.05)
    gate.update(io)
    # remote in use = refuse; mid-transition = refuse (shadow: the operator is driving, so both are expected)
    pre = gate.faults(io, since_t=io.now() - 2.0, need_running=not shadow)
    pose = gate.pose()
    if pose is not None:
        if start is not None:
            n, sx, sy, sh = start
            d = math.hypot(pose[0] - sx, pose[1] - sy)
            dh = abs(math.degrees(wrap(pose[2] - sh)))
            if d > P.start_tol_m or dh > P.start_tol_deg:
                pre.append(f'robot is not on the start waypoint {n}: localised {d:.2f} m / {dh:.0f} deg away '
                           f'(limits {P.start_tol_m} m / {P.start_tol_deg:.0f} deg) -- wrong spot, or wrong localisation')
        prev = pose[:2]
        for (name, wx, wy, _) in route:
            d = math.hypot(wx - prev[0], wy - prev[1])
            if d > MAX_LEG:
                pre.append(f'leg to {name} is {d:.1f} m > {MAX_LEG} m')
            prev = (wx, wy)
    return pre, pose


def run_route(io, P, route, align_final=False, shadow=False, log=print, row=None, start=None):
    """Walk `route` = [(name, x, y, heading_rad or None), ...] in order.

    io: poses_since/status_since/cmds_since/mode/send/now/stamp_now/sleep/stop_requested.
    Returns a result dict. Once preflight has passed, every exit (completion, abort, Ctrl-C,
    exception) ends with zero_tail_s of explicit zeros, and the abort is printed only AFTER
    them (a blocked terminal must not hold the zeros back).
    Shadow mode: the IO publishes nothing. A trip is logged as WOULD ABORT and the leg restarts
    from the current pose after a 2 s hold-off, so a whole driven route can be evaluated. Ctrl-C
    still ends it. Operator commands are expected (the operator drives) and do not trip."""
    assert 0.12 <= P.fwd_cmd <= MAX_FWD and 0.5 <= P.ang_turn <= MAX_ANG and 0 < P.ang_walk_max <= MAX_ANG
    assert P.arrive_trigger_m < P.near_m
    period = 1.0 / P.rate_hz
    gate = Gate(P, shadow)
    res = {'outcome': None, 'reason': None, 'arrivals': [], 'would_abort': [], 'sent_nonzero': False}
    t_start = io.now()
    sent = {'nonzero': False}

    def send(fwd, ang):
        if not (math.isfinite(fwd) and math.isfinite(ang)):
            raise Abort(f'controller produced a non-finite command ({fwd}, {ang})')
        fwd = clamp(fwd, 0.0, MAX_FWD)
        ang = clamp(ang, -MAX_ANG, MAX_ANG)
        if fwd != 0.0 or ang != 0.0:
            sent['nonzero'] = True
        io.send(fwd, ang)
        return fwd, ang

    def zeros(seconds):
        t_end = io.now() + seconds
        while io.now() < t_end:
            io.send(0.0, 0.0)
            io.sleep(period)

    def check():
        if io.stop_requested():
            raise Abort('stop requested (Ctrl-C / SIGTERM / SIGHUP / SIGQUIT)')
        gate.update(io)
        f = gate.faults(io, since)
        if f:
            raise Abort('; '.join(f))

    # ---- preflight: nothing is published unless all of this holds -----------------------
    pre, pose = preflight(io, gate, route, shadow, P.preflight_wait_s, start=start, P=P)
    if pre:
        res.update(outcome='REFUSED', reason='; '.join(pre))
        log('[preflight] REFUSING, nothing was published: ' + res['reason'])
        return res
    x, y, h = pose
    log(f'[preflight] OK: pose x {x:.2f} y {y:.2f} heading {math.degrees(h):.0f} deg, '
        f'{len(route)} waypoint(s){", SHADOW: nothing will be published" if shadow else ""}')

    since = io.now()                   # other sources' commands after this abort the run
    leg_i = 0
    deferred = None                    # abort message, printed only after the zero tail
    try:
        while leg_i < len(route):
            name, wx, wy, wh = route[leg_i]
            last = leg_i == len(route) - 1
            try:
                check()
                x, y, h = gate.pose()
                leg = Leg(P, name, (x, y), (wx, wy), wh if (align_final and last) else None, io.now())
                log(f'[leg {leg_i + 1}/{len(route)}] -> {name}: {leg.length:.2f} m, '
                    + (f'turn {math.degrees(wrap(math.atan2(wy - y, wx - x) - h)):+.0f} deg first'
                       if leg.state == 'TURN' else 'already there'))
                t_first_cmd, moving = None, False
                next_tick = io.now()
                while leg.state != 'ARRIVED':
                    check()
                    if gate.walking(io):
                        moving = True
                    x, y, h = gate.pose()
                    now = io.now()
                    fwd, ang, ev = leg.step(now, x, y, h, moving)
                    fwd, ang = send(fwd, ang)
                    if (fwd or ang) and t_first_cmd is None:
                        t_first_cmd = now
                    if (not shadow and t_first_cmd is not None and not moving
                            and now - t_first_cmd > P.start_wait_s):
                        raise Abort(f'mc did not start walking within {P.start_wait_s:.0f} s of our command')
                    if row:
                        dist, _, along, cross = leg.geometry(x, y)
                        md = io.mode()
                        fo = gate.foreign
                        row(dict(t=now, wall=io.stamp_now(), leg=leg_i + 1, name=name, state=leg.state, x=x, y=y,
                                 heading_deg=math.degrees(h), dist=dist, along=along, cross=cross, fwd=fwd,
                                 ang=ang, moving=int(moving), mode=f'{md[1]}/{md[2]}' if md else '',
                                 rejects=gate.rejects, rc_fwd=gate.rc_cmd[0], rc_lat=gate.rc_cmd[1],
                                 rc_ang=gate.rc_cmd[2], foreign_src=fo[1] if fo else '',
                                 foreign_age=(now - fo[0]) if fo else -1.0, event=ev or ''))
                    if ev:
                        dist = leg.geometry(x, y)[0]
                        log(f'  [{now - t_start:6.1f} s] {ev}: dist {dist:.2f} m, x {x:.2f} y {y:.2f} '
                            f'heading {math.degrees(h):.0f} deg')
                    next_tick += period
                    io.sleep(max(0.0, next_tick - io.now()))
                # ---- arrived: stop, let mc stand, measure where it actually stopped -----------
                zeros(P.zero_tail_s)
                if not shadow:
                    t_w = io.now()
                    while io.now() - t_w < P.stand_wait_s:
                        check()
                        md = io.mode()
                        if md and md[1] == 'STAND_DEFAULT' and md[2] == RUNNING:
                            break
                        io.sleep(0.05)
                pts, t_s = [], io.now()
                while io.now() - t_s < P.settle_s:
                    check()
                    pts.append(gate.pose())
                    io.sleep(0.05)
                ax = sum(p[0] for p in pts) / len(pts)
                ay = sum(p[1] for p in pts) / len(pts)
                ah = circ_mean([p[2] for p in pts])
                arr = dict(name=name, x=ax, y=ay, heading_deg=math.degrees(ah), err_m=math.hypot(ax - wx, ay - wy),
                           err_along=(ax - wx) * leg.ux + (ay - wy) * leg.uy,
                           err_left=(ay - wy) * leg.ux - (ax - wx) * leg.uy,
                           heading_err_deg=math.degrees(wrap(ah - wh)) if wh is not None else None)
                res['arrivals'].append(arr)
                log(f'  ARRIVED at {name}: {arr["err_m"]:.2f} m from the waypoint (ahead {arr["err_along"]:+.2f}, '
                    f'left {arr["err_left"]:+.2f}), heading {arr["heading_deg"]:.0f} deg')
                leg_i += 1
            except Abort as e:
                if not shadow or str(e).startswith('stop requested'):
                    raise
                log(f'  WOULD ABORT: {e}')
                res['would_abort'].append(str(e))
                t_h = io.now()
                while io.now() - t_h < 2.0:            # hold-off, then rebuild the leg from here
                    if io.stop_requested():
                        raise Abort('stop requested (Ctrl-C / SIGTERM / SIGHUP / SIGQUIT)')
                    gate.update(io)
                    io.sleep(0.05)
        res.update(outcome='COMPLETED')
    except Abort as e:
        res.update(outcome='ABORTED', reason=str(e))
        deferred = f'[abort] {e}'
    except BaseException as e:            # a bug: still stop, then let it surface
        res.update(outcome='ABORTED', reason=f'exception: {e!r}')
        deferred = f'[abort] exception: {e!r}'
        raise
    finally:
        zeros(P.zero_tail_s)              # FIRST, before any printing
        res['sent_nonzero'] = sent['nonzero']
        if deferred:
            try:
                log(deferred)
            except OSError:
                pass
    return res
