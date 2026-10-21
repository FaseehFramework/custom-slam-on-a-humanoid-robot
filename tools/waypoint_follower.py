#!/usr/bin/env python3
"""Waypoint follower: the ROS side. Runs ON soc1 (via tools/follow_robot.sh, which sets up
the vendor aimdk type support). All decisions live in follower_core.py, which is tested
offline against a simulated robot (tools/test_follower.py); this program is tested end to end
against a fake robot node (tools/test_follower_ros.sh). This file only moves data.

Subcommands:
  mark NAME     save the robot's current map pose (turning centre) as a waypoint. The robot
                must stand still. Publishes NOTHING.
  list          print the saved waypoints. No ROS.
  plan WP...    preflight + the legs it would walk. Publishes NOTHING.
  shadow WP...  run the whole follower while the operator drives; computes and logs commands
                but has NO publisher. Trips are logged as WOULD ABORT.
  go --from S WP...   walk the route. The ONLY subcommand that can move the robot.
                - the robot must stand on waypoint S (localisation must agree within 0.3 m /
                  15 deg), so a wrong-but-confident localisation cannot steer it
                - takes an exclusive motion lock (shared with loco_cmd_test.py) BEFORE asking
                  for GO, so two motion tools can never run at once
                - asks for GO on a terminal (or needs --confirm); every exit after preflight
                  ends in 1 s of zeros, sent before anything is printed

Inputs: /x2/localization/odometry + status (localizer_live); /aima/mc/locomotion/velocity
(all sources: the remote's heartbeat and takeovers; a 'pnc' message this process did not
send is reported as 'pnc-other' and stops the run); /aima/mc/leg_odometry (BEST_EFFORT, for
the after-stop check, independent of localisation); GetMcAction polled at 5 Hz in its own
thread. Output (go only): McLocomotionVelocity as source 'pnc', BEST_EFFORT + VOLATILE (no
latching), clamped to the hard caps a second time. Never calls SetMcAction /
SetMcInputSource, never publishes as rc.
"""
import argparse, csv, fcntl, hashlib, json, math, os, queue, re, signal, sys, threading, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follower_core as fc

POSE_TOPIC = '/x2/localization/odometry'
STATUS_TOPIC = '/x2/localization/status'
CMD_TOPIC = '/aima/mc/locomotion/velocity'
LEG_TOPIC = '/aima/mc/leg_odometry'
ACTION_SRV = '/aimdk_5Fmsgs/srv/GetMcAction'
# X2_* overrides exist for tools/test_follower_ros.py (laptop, private DDS domain) only
MAPS = os.environ.get('X2_MAPS', '/home/run/x2_ultra_slam/maps')
LOGS = os.environ.get('X2_LOGS', os.path.expanduser('~/slam_ws'))
PIDFILE = os.environ.get('X2_PIDFILE', '/tmp/x2_waypoint_follower.pid')
LOCKFILE = os.environ.get('X2_LOCKFILE', '/tmp/x2_motion.lock')      # shared with loco_cmd_test.py
LOCALIZER_LOG = os.environ.get('X2_LOCALIZER_LOG', '/tmp/x2_localizer.log')
NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,40}$')


# ---- waypoint file and map identity -------------------------------------------------------
def wp_path(map_name):
    return os.path.join(MAPS, f'{map_name}_waypoints.json')


def map_file(map_name):
    return os.path.join(MAPS, f'{map_name}.pcd')


def sha1(path):
    h = hashlib.sha1()
    with open(path, 'rb') as f:
        for b in iter(lambda: f.read(1 << 20), b''):
            h.update(b)
    return h.hexdigest()


def load_wps(map_name):
    p = wp_path(map_name)
    if not os.path.exists(p):
        return {'map': map_name, 'map_sha1': None, 'waypoints': []}
    with open(p) as f:
        d = json.load(f)
    if d.get('map') != map_name:
        sys.exit(f'{p} is for map {d.get("map")!r}, not {map_name!r}')
    return d


def save_wps(d):
    p = wp_path(d['map'])
    tmp = p + '.tmp'
    with open(tmp, 'w') as f:
        json.dump(d, f, indent=1)
    os.replace(tmp, p)


def check_map(map_name, wps_doc):
    """The running localizer must use MAPS/<map>.pcd (full path, from its log), and that file
    must be the one the waypoints were captured on (sha1). Returns (error or None, sha1)."""
    try:
        txt = open(LOCALIZER_LOG).read()
    except OSError:
        return f'cannot read {LOCALIZER_LOG}: is the localizer running (live_slam.sh --localize)?', None
    m = re.findall(r'map (\S+\.pcd) \(', txt)
    if not m:
        return f'{LOCALIZER_LOG} names no map', None
    want = map_file(map_name)
    if os.path.realpath(m[-1]) != os.path.realpath(want):
        return f'localizer is running on {m[-1]}, waypoints are for {want}', None
    try:
        h = sha1(want)
    except OSError as e:
        return f'cannot read {want}: {e}', None
    if wps_doc.get('map_sha1') and wps_doc['map_sha1'] != h:
        return (f'{want} has changed since these waypoints were captured (sha1 {h[:10]} vs '
                f'{wps_doc["map_sha1"][:10]}): their coordinates belong to the old map. Move '
                f'{wp_path(map_name)} aside and capture again.'), h
    return None, h


def resolve(map_name, names):
    wps = {w['name']: w for w in load_wps(map_name)['waypoints']}
    missing = [n for n in names if n not in wps]
    if missing:
        sys.exit(f'unknown waypoint(s) {missing}; saved: {sorted(wps)}')
    return [(n, wps[n]['x'], wps[n]['y'], math.radians(wps[n]['heading_deg'])) for n in names]


# ---- logging that cannot block the control thread -----------------------------------------
class Printer:
    """print() from a daemon thread: a stopped (Ctrl-S) or hung-up terminal can never hold
    back a command. Also appends every line to a log file (flushed per line)."""

    def __init__(self, path=None):
        self.q = queue.Queue()
        self.f = open(path, 'a', buffering=1) if path else None
        threading.Thread(target=self._run, daemon=True).start()

    def __call__(self, s):
        self.q.put(s)

    def _run(self):
        while True:
            s = self.q.get()
            if s is None:
                return
            if self.f:
                try:
                    self.f.write(s + '\n')
                except OSError:
                    pass
            try:
                print(s, flush=True)
            except OSError:
                pass                                 # the terminal is gone; the file has it

    def close(self, timeout=2.0):
        self.q.put(None)
        t0 = time.monotonic()
        while not self.q.empty() and time.monotonic() - t0 < timeout:
            time.sleep(0.02)


# ---- ROS ----------------------------------------------------------------------------------
class Log:
    """append-only list with a moving start, so long runs do not grow without bound"""

    def __init__(self, keep=5000):
        self.items, self.base, self.keep = [], 0, keep

    def add(self, x):
        self.items.append(x)
        if len(self.items) > 2 * self.keep:
            drop = len(self.items) - self.keep
            del self.items[:drop]
            self.base += drop

    def since(self, i):
        j = max(i, self.base) - self.base
        return self.items[j:], self.base + len(self.items)


def make_io(with_mc, publish):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    be = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE)

    class RosIO(Node):
        def __init__(self):
            super().__init__('x2_waypoint_follower')
            self.lock = threading.Lock()
            self.poses, self.status, self.cmds, self.legs = Log(), Log(), Log(20000), Log(5000)
            self.mode_v = None
            self.stop = threading.Event()        # a stop was requested (signal)
            self.quit = threading.Event()        # threads should exit (shutdown)
            self.pub = None
            self.seq = 0
            self.mine = {}                       # header stamps we sent -> None (insertion-ordered)
            self.create_subscription(Odometry, POSE_TOPIC, self.on_pose, rel)
            self.create_subscription(String, STATUS_TOPIC, self.on_status, rel)
            if with_mc:
                from aimdk_msgs.msg import McLocomotionVelocity
                from aimdk_msgs.srv import GetMcAction
                self.Msg, self.Req = McLocomotionVelocity, GetMcAction.Request
                self.create_subscription(McLocomotionVelocity, CMD_TOPIC, self.on_cmd, be)
                self.create_subscription(Odometry, LEG_TOPIC, self.on_leg, be)   # BEST_EFFORT: rule 7
                self.cli = self.create_client(GetMcAction, ACTION_SRV)
                if publish:
                    self.pub = self.create_publisher(McLocomotionVelocity, CMD_TOPIC, be)

        # callbacks (executor thread)
        def on_pose(self, m):
            p, q, s = m.pose.pose.position, m.pose.pose.orientation, m.header.stamp
            with self.lock:
                self.poses.add((time.monotonic(), s.sec + s.nanosec * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w))

        def on_status(self, m):
            with self.lock:
                self.status.add((time.monotonic(), m.data))

        def on_cmd(self, m):
            src = m.source
            with self.lock:
                if src == fc.SOURCE and (m.header.stamp.sec, m.header.stamp.nanosec) not in self.mine:
                    src = 'pnc-other'            # someone else publishing as pnc: not us
                self.cmds.add((time.monotonic(), src, m.forward_velocity, m.lateral_velocity, m.angular_velocity))

        def on_leg(self, m):
            p = m.pose.pose.position
            with self.lock:
                self.legs.add((time.monotonic(), p.x, p.y))

        # mode poller (own thread): GetMcAction at 5 Hz; a stale value is a fault in the Gate
        def poll_modes(self):
            while not self.quit.is_set() and rclpy.ok():
                t0 = time.monotonic()
                try:
                    if self.cli.service_is_ready() or self.cli.wait_for_service(timeout_sec=0.5):
                        fut = self.cli.call_async(self.Req())
                        while not fut.done() and time.monotonic() - t0 < 0.5:
                            time.sleep(0.01)
                        if fut.done() and fut.result() is not None:
                            r = fut.result()
                            with self.lock:
                                self.mode_v = (time.monotonic(), r.info.action_desc, r.info.status.value)
                        elif not fut.done():
                            fut.cancel()
                except Exception:
                    pass                         # shutting down; a stale mode is a fault anyway
                time.sleep(max(0.0, 0.2 - (time.monotonic() - t0)))

        # follower_core IO interface
        def now(self):
            return time.monotonic()

        def stamp_now(self):
            return time.time()           # PTP-disciplined; header stamps are the same clock

        def sleep(self, dt):
            time.sleep(dt)

        def stop_requested(self):
            return self.stop.is_set()

        def mode(self):
            with self.lock:
                return self.mode_v

        def poses_since(self, i):
            with self.lock:
                return self.poses.since(i)

        def status_since(self, i):
            with self.lock:
                return self.status.since(i)

        def cmds_since(self, i):
            with self.lock:
                return self.cmds.since(i)

        def send(self, fwd, ang):
            if self.pub is None:         # plan / shadow: there is no publisher at all
                return
            fwd = min(max(float(fwd), 0.0), fc.MAX_FWD)          # hard caps, second time
            ang = min(max(float(ang), -fc.MAX_ANG), fc.MAX_ANG)
            if not (math.isfinite(fwd) and math.isfinite(ang)):
                fwd = ang = 0.0
            m = self.Msg()
            now = self.get_clock().now().to_msg()
            m.header.stamp = now
            m.header.meas_stamp = now
            m.header.frame_id = 'locomotion_velocity'
            m.header.sequence = self.seq
            self.seq += 1
            m.source = fc.SOURCE
            m.forward_velocity = fwd
            m.lateral_velocity = 0.0
            m.angular_velocity = ang
            with self.lock:              # remember it BEFORE publishing: our echo must match
                self.mine[(now.sec, now.nanosec)] = None
                if len(self.mine) > 2000:
                    for k in list(self.mine)[:1000]:
                        del self.mine[k]
            self.pub.publish(m)

    rclpy.init()
    io = RosIO()
    io.threads = [threading.Thread(target=rclpy.spin, args=(io,), daemon=True)]
    if with_mc:
        io.threads.append(threading.Thread(target=io.poll_modes, daemon=True))
    for t in io.threads:
        t.start()
    return io, rclpy


def shutdown(io, rclpy):
    """publisher first, then ROS, then JOIN the threads: exiting with rclpy.spin still inside
    C++ aborts the interpreter ('terminate called without an active exception')."""
    io.quit.set()
    if io.pub is not None:
        io.destroy_publisher(io.pub)
        io.pub = None
    try:
        rclpy.shutdown()
    except Exception:
        pass
    for t in io.threads:
        t.join(timeout=2.0)


# ---- subcommands --------------------------------------------------------------------------
def cmd_list(a):
    d = load_wps(a.map)
    print(f'{wp_path(a.map)}: {len(d["waypoints"])} waypoint(s), map sha1 {(d.get("map_sha1") or "?")[:10]}')
    prev = None
    for w in d['waypoints']:
        step = f'{math.hypot(w["x"] - prev["x"], w["y"] - prev["y"]):5.2f} m from {prev["name"]}' if prev else ''
        print(f'  {w["name"]:<16} x {w["x"]:7.2f} y {w["y"]:7.2f} heading {w["heading_deg"]:6.1f} deg  '
              f'({w["captured"]}, spread {w["spread_m"] * 100:.1f} cm)  {step}')
        prev = w
    return 0


def cmd_mark(a):
    if not NAME_RE.match(a.name):
        sys.exit('waypoint names: letters, digits, _ and - only (max 40)')
    d = load_wps(a.map)
    if any(w['name'] == a.name for w in d['waypoints']) and not a.force:
        sys.exit(f'{a.name!r} exists; --force replaces it')
    bad, h = check_map(a.map, d)
    if bad:
        sys.exit(f'refusing: {bad}')
    io, rclpy = make_io(with_mc=False, publish=False)
    try:
        P = fc.Params()
        gate = fc.Gate(P)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 8.0:      # discovery, then a TRACKING status whose last match was OK
            gate.update(io)
            if gate.status and gate.status.startswith('TRACKING') and gate.rejects == 0 and gate.pose():
                break
            time.sleep(0.05)
        else:
            sys.exit(f'refusing: localiser not tracking with an accepted match (status: {gate.status!r})')
        pts, t0 = [], time.monotonic()
        while time.monotonic() - t0 < a.seconds:
            n0 = gate.pi
            gate.update(io)
            if gate.pi != n0 and gate.last_pose is not None:
                tr, st, x, y, z, hh = gate.last_pose
                pts.append((x, y, z, hh))
            time.sleep(0.02)
        if gate.nonfinite or time.monotonic() - gate.last_pose[0] > P.pose_stale_s or len(pts) < 5 * a.seconds:
            sys.exit(f'refusing: only {len(pts)} usable poses in {a.seconds} s')
        mx = sum(p[0] for p in pts) / len(pts); my = sum(p[1] for p in pts) / len(pts)
        mz = sum(p[2] for p in pts) / len(pts)
        mh = fc.circ_mean([p[3] for p in pts])
        spread = max(math.hypot(p[0] - mx, p[1] - my) for p in pts)
        hspread = max(abs(math.degrees(fc.wrap(p[3] - mh))) for p in pts)
        if spread > 0.03 or hspread > 3.0:
            sys.exit(f'refusing: robot not still (spread {spread * 100:.1f} cm, {hspread:.1f} deg)')
        w = dict(name=a.name, x=round(mx, 4), y=round(my, 4), z=round(mz, 4), heading_deg=round(math.degrees(mh), 2),
                 captured=time.strftime('%Y-%m-%d %H:%M:%S'), samples=len(pts), spread_m=round(spread, 4),
                 point='turning centre (chest - %.2f m forward)' % fc.BASE_FWD)
        d['map_sha1'] = h
        d['map_path'] = map_file(a.map)
        d['waypoints'] = [x for x in d['waypoints'] if x['name'] != a.name] + [w]
        save_wps(d)
        print(f'saved {a.name}: x {mx:.3f} y {my:.3f} heading {math.degrees(mh):.1f} deg '
              f'({len(pts)} poses, spread {spread * 100:.1f} cm / {hspread:.1f} deg) -> {wp_path(a.map)}')
        return 0
    finally:
        shutdown(io, rclpy)


def print_plan(pose, route):
    x, y, h = pose
    for i, (n, wx, wy, wh) in enumerate(route):
        b = math.atan2(wy - y, wx - x)
        d = math.hypot(wx - x, wy - y)
        turn = math.degrees(fc.wrap(b - h))
        print(f'  leg {i + 1}: -> {n:<14} {d:5.2f} m' + (
            f', turn {abs(turn):3.0f} deg {"left" if turn > 0 else "right"} first' if d >= fc.Params().arrive_trigger_m
            else ', already there'))
        x, y, h = wx, wy, b


def cmd_plan(a):
    route = resolve(a.map, a.waypoints)
    start = resolve(a.map, [a.start])[0] if a.start else None
    bad, _ = check_map(a.map, load_wps(a.map))
    io, rclpy = make_io(with_mc=True, publish=False)
    try:
        P = fc.Params()
        pre, pose = fc.preflight(io, fc.Gate(P), route, wait_s=P.preflight_wait_s, start=start, P=P)
        if bad:
            pre.insert(0, bad)
        if pose:
            print(f'robot (turning centre) at x {pose[0]:.2f} y {pose[1]:.2f} heading {math.degrees(pose[2]):.0f} deg')
            print_plan(pose, route)
        print('preflight: ' + ('OK' if not pre else 'WOULD REFUSE: ' + '; '.join(pre)))
        return 0 if not pre else 3
    finally:
        shutdown(io, rclpy)


def take_motion_lock():
    """Exclusive, held for the life of the process; the kernel drops it on ANY death, so a
    stale file can never block, and two motion tools can never both hold it."""
    fd = os.open(LOCKFILE, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        holder = os.pread(fd, 64, 0).decode(errors='replace').strip()
        sys.exit(f'refusing: another motion tool is running (lock {LOCKFILE} held by {holder or "?"})')
    os.ftruncate(fd, 0)
    os.pwrite(fd, f'pid {os.getpid()} waypoint_follower'.encode(), 0)
    return fd


def stop_check(io, since_cmd_i, seconds=4.0):
    """Did the robot stop? Leg odometry (independent of localisation) over `seconds`; speed
    over the second half. Returns (verdict, speed or None)."""
    with io.lock:
        _, li = io.legs.since(0)
    time.sleep(seconds)
    with io.lock:
        legs, _ = io.legs.since(li)
        cmds, _ = io.cmds.since(since_cmd_i)
    driving = any(src != fc.SOURCE and abs(f) + abs(l) + abs(a) > 1e-3 for (_, src, f, l, a) in cmds)
    t_end = time.monotonic()
    late = [p for p in legs if p[0] >= t_end - seconds / 2]
    if len(late) < 10 or t_end - late[-1][0] > 0.5:
        return 'UNKNOWN (no fresh leg odometry)', None
    v = math.hypot(late[-1][1] - late[0][1], late[-1][2] - late[0][2]) / max(1e-3, late[-1][0] - late[0][0])
    if driving:
        return 'operator is driving (not judged)', v
    return ('STILL MOVING' if v > 0.05 else 'stopped'), v


def cmd_run(a, shadow):
    route = resolve(a.map, a.waypoints)
    start = resolve(a.map, [a.start])[0] if a.start else None
    P = fc.Params()
    bad, _ = check_map(a.map, load_wps(a.map))
    if bad:
        sys.exit(f'refusing: {bad}')
    lock_fd = None
    if not shadow:
        lock_fd = take_motion_lock()           # BEFORE the prompt: nobody else can start meanwhile
        print(f'GO: from {a.start}, walk {" -> ".join(a.waypoints)}{" and align" if a.align_final else ""} '
              f'.\n  mc walks at ~0.45 m/s (its slowest walk) and turns in place at ~0.5-1.0 rad/s.')
        print('  The operator holds the controller: ANY stick input cancels the run. Clear space, e-stop ready.')
        if not a.confirm:
            if not sys.stdin.isatty():
                sys.exit('refusing: not a terminal, and no --confirm')
            if input('  Type GO to start: ').strip() != 'GO':
                sys.exit('not started; nothing was published')
    stamp = time.strftime('%Y%m%d_%H%M%S') + f'_{os.getpid()}'
    base = os.path.join(LOGS, f'{"shadow" if shadow else "follow"}_{stamp}')
    out = Printer(base + '.log')
    io, rclpy = make_io(with_mc=True, publish=not shadow)
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP, signal.SIGQUIT):
        signal.signal(sig, lambda *_: io.stop.set())
    signal.signal(signal.SIGTSTP, signal.SIG_IGN)            # Ctrl-Z must not freeze a walking robot
    if not shadow:
        with open(PIDFILE, 'w') as f:
            f.write(str(os.getpid()))
    meta = dict(route=a.waypoints, start=a.start, map=a.map, params=vars(P), shadow=shadow, pid=os.getpid(),
                wall_start=time.time(), mono_start=time.monotonic())
    with open(base + '.json', 'w') as f:                      # a killed run still leaves a record
        json.dump(dict(meta, outcome='RUNNING'), f, indent=1, default=str)
    fcsv = open(base + '.csv', 'w', newline='', buffering=1)
    w = None

    def row(d):
        nonlocal w
        if w is None:
            w = csv.DictWriter(fcsv, fieldnames=list(d))
            w.writeheader()
        w.writerow({k: (f'{v:.4f}' if isinstance(v, float) else v) for k, v in d.items()})

    res = None
    try:
        res = fc.run_route(io, P, route, align_final=a.align_final, shadow=shadow, row=row, log=out, start=start)
    finally:
        if io.pub is not None:
            io.destroy_publisher(io.pub)      # nothing can be published after this line
            io.pub = None
        fcsv.close()
        with io.lock:
            _, ci = io.cmds.since(0)
        verdict, v = stop_check(io, max(0, ci - 1))
        if res is not None:
            res.update(stop_check=verdict, speed_after_stop=v)
            with open(base + '.json', 'w') as f:
                json.dump(dict(meta, **res), f, indent=1, default=str)
        out(f'\n[result] {res["outcome"] if res else "EXCEPTION"}' + (f': {res["reason"]}' if res and res.get('reason') else ''))
        for arr in (res or {}).get('arrivals', []):
            out(f'  {arr["name"]:<14} {arr["err_m"]:.2f} m from the waypoint (ahead {arr["err_along"]:+.2f}, '
                f'left {arr["err_left"]:+.2f})')
        if res and res.get('would_abort'):
            out(f'  shadow: {len(res["would_abort"])} WOULD ABORT event(s)')
        out(f'  after the stop: {verdict}' + (f', {v:.3f} m/s (leg odometry)' if v is not None else ''))
        if verdict == 'STILL MOVING' and not shadow:
            out('\n  !!! ROBOT STILL MOVING AFTER THE STOP -- use the remote e-stop. !!!')
        out(f'  logs: {base}.csv / .json / .log')
        out.close()
        try:
            if open(PIDFILE).read().strip() == str(os.getpid()):
                os.remove(PIDFILE)
        except OSError:
            pass
        shutdown(io, rclpy)
        if lock_fd is not None:
            os.close(lock_fd)
    return 0 if res['outcome'] == 'COMPLETED' else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--map', default='map01')
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('mark'); s.add_argument('name'); s.add_argument('--force', action='store_true')
    s.add_argument('--seconds', type=float, default=2.0)
    sub.add_parser('list')
    s = sub.add_parser('plan'); s.add_argument('waypoints', nargs='+')
    s.add_argument('--from', dest='start', help='waypoint the robot stands on (checked)')
    for name in ('shadow', 'go'):
        s = sub.add_parser(name); s.add_argument('waypoints', nargs='+')
        s.add_argument('--from', dest='start', required=(name == 'go'),
                       help='waypoint the robot stands on; localisation must agree (0.3 m / 15 deg)')
        s.add_argument('--align-final', action='store_true', help="turn to the last waypoint's saved heading")
        if name == 'go':
            s.add_argument('--confirm', action='store_true', help='skip the typed GO (operator ready)')
    a = ap.parse_args()
    if a.cmd == 'list':
        return cmd_list(a)
    if a.cmd == 'mark':
        return cmd_mark(a)
    if a.cmd == 'plan':
        return cmd_plan(a)
    return cmd_run(a, shadow=(a.cmd == 'shadow'))


if __name__ == '__main__':
    sys.exit(main())
