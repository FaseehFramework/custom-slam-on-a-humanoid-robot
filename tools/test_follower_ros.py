#!/usr/bin/env python3
"""End-to-end test of tools/waypoint_follower.py, the REAL ROS program, on the laptop.

A fake robot node runs the simulator from test_follower.py in real time and speaks the
robot's topics and service: /x2/localization/odometry + status, the remote's ~50 Hz `rc`
stream on /aima/mc/locomotion/velocity, /aima/mc/leg_odometry, GetMcAction, and it consumes
our `pnc` commands (mc's 1 s expiry included). The follower runs as a subprocess, exactly as on
soc1.

ISOLATION: run it through tools/test_follower_ros.sh, which sets ROS_LOCALHOST_ONLY=1 and a
private ROS_DOMAIN_ID and puts the LAPTOP-ONLY fake aimdk_msgs (test_ws) on the path.
Nothing leaves this machine. ~6 min.
"""
import json, math, os, signal, subprocess, sys, tempfile, threading, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import follower_core as fc
from test_follower import Sim, RULE_FWD, RULE_ANG

assert os.environ.get('ROS_LOCALHOST_ONLY') == '1', 'run via tools/test_follower_ros.sh (isolation)'

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from aimdk_msgs.msg import McLocomotionVelocity
from aimdk_msgs.srv import GetMcAction

HERE = os.path.dirname(os.path.abspath(__file__))
REL = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE)
BE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_x2')
        self.lock = threading.Lock()
        self.sim = None
        self.pose_pub = self.create_publisher(Odometry, '/x2/localization/odometry', REL)
        self.status_pub = self.create_publisher(String, '/x2/localization/status', REL)
        self.leg_pub = self.create_publisher(Odometry, '/aima/mc/leg_odometry', BE)
        self.cmd_pub = self.create_publisher(McLocomotionVelocity, '/aima/mc/locomotion/velocity', BE)
        self.create_subscription(McLocomotionVelocity, '/aima/mc/locomotion/velocity', self.on_cmd, BE)
        self.create_service(GetMcAction, '/aimdk_5Fmsgs/srv/GetMcAction', self.on_action)
        self.create_timer(0.01, self.tick)

    def reset(self, sim, other_pnc_at=None):
        with self.lock:
            self.sim = sim
            self.wall0 = time.time()
            self.mono0 = time.monotonic()
            self.ip = self.is_ = self.ic = 0
            self.pnc = []               # (wall, fwd, lat, ang) received from the follower
            self.other_pnc_at = other_pnc_at

    def on_cmd(self, m):
        if m.source != 'pnc' or m.header.frame_id == 'other':
            return                      # our own rc echo / our impersonated other pnc publisher
        with self.lock:
            if self.sim is None:
                return
            self.pnc.append((time.time(), m.forward_velocity, m.lateral_velocity, m.angular_velocity))
            self.sim.pnc_cmds.append((self.sim.t, m.forward_velocity, m.angular_velocity))

    def on_action(self, req, resp):
        with self.lock:
            if self.sim is not None:
                resp.info.action_desc, resp.info.status.value = self.sim.mode_v
        return resp

    def tick(self):
        with self.lock:
            s = self.sim
            if s is None:
                return
            target = time.monotonic() - self.mono0
            if target > s.t:
                s.sleep(target - s.t)
            for (tr, st, x, y, z, qx, qy, qz, qw) in s.poses[self.ip:]:
                m = Odometry()
                ts = self.wall0 + st
                m.header.stamp.sec = int(ts); m.header.stamp.nanosec = int((ts - int(ts)) * 1e9)
                m.header.frame_id = 'map'
                m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = x, y, z
                o = m.pose.pose.orientation
                o.x, o.y, o.z, o.w = qx, qy, qz, qw
                self.pose_pub.publish(m)
            self.ip = len(s.poses)
            for (tr, txt) in s.status[self.is_:]:
                self.status_pub.publish(String(data=txt))
            self.is_ = len(s.status)
            for (tr, src, f, l, a) in s.cmds[self.ic:]:
                if src == 'rc':
                    m = McLocomotionVelocity()
                    m.source, m.forward_velocity, m.lateral_velocity, m.angular_velocity = 'rc', f, l, a
                    self.cmd_pub.publish(m)
            self.ic = len(s.cmds)
            leg = Odometry()                       # leg odometry: the true turning centre, 100 Hz
            leg.pose.pose.position.x, leg.pose.pose.position.y = s.x, s.y
            self.leg_pub.publish(leg)
            if self.other_pnc_at is not None and self.other_pnc_at <= s.t < self.other_pnc_at + 1.0:
                m = McLocomotionVelocity()         # ANOTHER program publishing as pnc
                m.header.frame_id = 'other'
                m.header.stamp.sec = 7; m.header.stamp.nanosec = int(s.t * 1e6)
                m.source, m.forward_velocity = 'pnc', 0.1
                self.cmd_pub.publish(m)


def follower(args, env, **kw):
    return subprocess.Popen([sys.executable, '-u', os.path.join(HERE, 'waypoint_follower.py'), '--map', 'testmap'] + args,
                            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, **kw)


def main():
    rclpy.init()
    robot = FakeRobot()
    ex = SingleThreadedExecutor()
    ex.add_node(robot)
    spin_t = threading.Thread(target=ex.spin, daemon=True)
    spin_t.start()
    tmp = tempfile.mkdtemp(prefix='x2follow_')
    env = dict(os.environ, X2_MAPS=tmp, X2_LOGS=tmp, X2_PIDFILE=os.path.join(tmp, 'pid'),
               X2_LOCKFILE=os.path.join(tmp, 'motion.lock'), X2_LOCALIZER_LOG=os.path.join(tmp, 'loc.log'))
    mapf = os.path.join(tmp, 'testmap.pcd')
    with open(mapf, 'w') as f:
        f.write('# fake map v1\n')
    with open(env['X2_LOCALIZER_LOG'], 'w') as f:
        f.write(f'[INFO] [0] [x2_localizer_live]: map {mapf} (1 pts), prior x 0.00\n')
    fails = []

    def report(name, bad, out):
        tag = 'ok  ' if not bad else 'FAIL'
        last = [l for l in out.splitlines() if l.startswith('[result]') or 'REFUS' in l or 'refusing' in l or 'saved' in l]
        print(f'{tag} {name:<46} {(last[-1] if last else out.strip().splitlines()[-1] if out.strip() else "")[:100]}')
        for b in bad:
            print(f'       !! {b}')
        if bad:
            fails.append(name)
            print('       --- follower output ---\n' + '\n'.join('       ' + l for l in out.splitlines()[-25:]))

    def cmd_checks(expect_nonzero=True):
        bad = []
        with robot.lock:
            pnc = list(robot.pnc)
        for (t, f, l, a) in pnc:
            if not (0 <= f <= RULE_FWD and l == 0 and abs(a) <= RULE_ANG):
                bad.append(f'cap violated: fwd {f} lat {l} ang {a}'); break
        nz = [i for i, p in enumerate(pnc) if p[1] or p[3]]
        if expect_nonzero and not nz:
            bad.append('no non-zero command ever arrived')
        if pnc and nz:
            t0 = pnc[nz[-1]][0]
            first = [p[0] for p in pnc[nz[-1] + 1:] if p[0] <= t0 + 1.0]
            times = [t0] + first
            gap = max((b - a for a, b in zip(times, times[1:])), default=9.0)
            if len(first) < 15 or times[-1] - t0 < 0.9 or gap > 0.15:
                bad.append(f'zero tail: {len(first)} msgs over {times[-1] - t0:.2f} s, max gap {gap:.2f} s')
        if pnc and (pnc[-1][1] or pnc[-1][3]):
            bad.append('last command received was non-zero')
        return bad, pnc

    def settle_check(bad, sim_wait=4.0):
        time.sleep(sim_wait)
        with robot.lock:
            s = robot.sim
            if abs(s.v) > 0.01 or abs(s.w) > 0.01:
                bad.append(f'simulated robot still moving: v {s.v:.3f} w {s.w:.3f}')

    def run_go(name, sim_kw, route_args, inject=None, expect='COMPLETED', expect_text=None, expect_nonzero=True,
               extra=(), start='home', other_pnc_at=None):
        robot.reset(Sim(seed=5, **sim_kw), other_pnc_at=other_pnc_at)
        time.sleep(0.5)
        p = follower(['go', '--from', start] + route_args + ['--confirm'] + list(extra), env)
        if inject:
            inject(p)
        out, _ = p.communicate(timeout=180)
        bad, pnc = cmd_checks(expect_nonzero)
        if f'[result] {expect}' not in out and not (expect == 'REFUSED' and 'REFUSING' in out):
            bad.append(f'expected [result] {expect}')
        if expect_text and expect_text not in out:
            bad.append(f'output lacks {expect_text!r}')
        if 'STILL MOVING' in out:
            bad.append('false "STILL MOVING" alarm')
        if expect == 'REFUSED' and pnc:
            bad.append(f'refused but {len(pnc)} commands were published')
        if os.path.exists(env['X2_PIDFILE']):
            bad.append('pid file left behind')
        settle_check(bad)
        report(name, bad, out)
        return out

    # ---- waypoints: mark at known poses -------------------------------------------------
    def mark(name, x, y, yaw):
        robot.reset(Sim(seed=2, x=x, y=y, yaw=yaw))
        time.sleep(1.0)
        p = follower(['mark', name], env)
        out, _ = p.communicate(timeout=60)
        bad = [] if p.returncode == 0 else [f'exit {p.returncode}']
        with robot.lock:
            if robot.pnc:
                bad.append(f'mark published {len(robot.pnc)} commands')
        wf = os.path.join(tmp, 'testmap_waypoints.json')
        doc = json.load(open(wf)) if os.path.exists(wf) else {'waypoints': []}
        wps = {w['name']: w for w in doc['waypoints']}
        w = wps.get(name)
        if not doc.get('map_sha1'):
            bad.append('map sha1 not stored')
        if not w:
            bad.append('not saved')
        else:
            if math.hypot(w['x'] - x, w['y'] - y) > 0.03:     # the TURNING CENTRE, not the chest
                bad.append(f'saved at ({w["x"]:.3f}, {w["y"]:.3f}), robot centre at ({x:.3f}, {y:.3f})')
            if abs(fc.wrap(math.radians(w['heading_deg']) - yaw)) > math.radians(2):
                bad.append(f'heading {w["heading_deg"]:.1f} vs {math.degrees(yaw):.1f}')
        report(f'mark {name}', bad, out)

    mark('home', 0.0, 0.0, 0.0)
    mark('a', 3.0, 0.0, 0.0)        # 3 m legs: faults injected at 6 s land mid-walk
    mark('b', 3.0, 2.0, math.pi / 2)

    p = follower(['list'], env); out, _ = p.communicate(timeout=30)
    report('list', [] if ('home' in out and ' b ' in out and p.returncode == 0) else ['list output wrong'], out)

    robot.reset(Sim(seed=3))
    time.sleep(0.5)
    p = follower(['plan', 'a', 'b', '--from', 'home'], env); out, _ = p.communicate(timeout=60)
    bad = [] if (p.returncode == 0 and 'leg 2' in out and 'preflight: OK' in out) else [f'exit {p.returncode}']
    with robot.lock:
        if robot.pnc:
            bad.append('plan published commands')
    report('plan --from home a b', bad, out)

    # ---- go ------------------------------------------------------------------------------
    run_go('go a b --align-final (nominal)', {}, ['a', 'b'], extra=['--align-final'], expect_text='after the stop: stopped')

    def sig_at(sig, t):
        def f(p):
            time.sleep(t); p.send_signal(sig)
        return f
    for sig, nm in ((signal.SIGINT, 'SIGINT (Ctrl-C)'), (signal.SIGHUP, 'SIGHUP (terminal closed)'),
                    (signal.SIGTERM, 'SIGTERM'), (signal.SIGQUIT, 'SIGQUIT (Ctrl-\\)')):
        run_go(f'go a: {nm} at 6 s', {}, ['a'], inject=sig_at(sig, 6.0), expect='ABORTED', expect_text='stop requested')
    run_go('go a: operator flicks forward', dict(rc_flick_at=6.0), ['a'], expect='ABORTED', expect_text='operator took over')
    run_go('go a: operator flicks turn only', dict(rc_flick_at=6.0, rc_flick=(0.0, 0.0, 0.5)), ['a'], expect='ABORTED',
           expect_text='operator took over')
    run_go('go a: another program publishes pnc', {}, ['a'], other_pnc_at=6.0, expect='ABORTED',
           expect_text='another command source')
    run_go('go a: localisation freezes', dict(pose_freeze_at=6.0), ['a'], expect='ABORTED', expect_text='pose stale')
    run_go('go a: controller disconnects', dict(rc_off_at=6.0), ['a'], expect='ABORTED', expect_text='remote controller')
    run_go('go a: e-stop', dict(estop_at=6.0), ['a'], expect='ABORTED', expect_text='motion mode')
    run_go('go: refused, robot not on --from', dict(x=0.6), ['a'], expect='REFUSED', expect_nonzero=False,
           expect_text='not on the start waypoint')

    # simulator-model check, NOT evidence about the real mc: SIGKILL means no zeros at all;
    # only the fake mc's 1 s expiry stops it here. Stage C tests the real one.
    robot.reset(Sim(seed=5))
    time.sleep(0.5)
    p = follower(['go', '--from', 'home', 'a', '--confirm'], env)
    time.sleep(6.0)
    p.kill(); p.communicate(timeout=30)
    bad = []
    time.sleep(3.5)
    with robot.lock:
        v, w = robot.sim.v, robot.sim.w
    if abs(v) > 0.01 or abs(w) > 0.01:
        bad.append(f'sim robot still moving 3.5 s after SIGKILL: v {v:.3f}')
    report('go a: SIGKILL (sim mc expiry only)', bad, 'killed')
    if os.path.exists(env['X2_PIDFILE']):
        os.remove(env['X2_PIDFILE'])    # a SIGKILL leaves it (only follow.sh stop reads it, and checks the pid)
    run_go('go after a SIGKILL (stale lock file)', {}, ['a'])

    # not a terminal and no --confirm: must refuse, publish nothing
    robot.reset(Sim(seed=5))
    time.sleep(0.5)
    p = follower(['go', '--from', 'home', 'a'], env, stdin=subprocess.DEVNULL)
    out, _ = p.communicate(timeout=60)
    bad = []
    with robot.lock:
        if robot.pnc:
            bad.append(f'{len(robot.pnc)} commands published')
    if 'not a terminal' not in out:
        bad.append('did not refuse for lack of a terminal')
    report('go without terminal or --confirm', bad, out)

    # two followers started 0.05 s apart: exactly one may run
    robot.reset(Sim(seed=5))
    time.sleep(0.5)
    p1 = follower(['go', '--from', 'home', 'a', '--confirm'], env)
    time.sleep(0.05)
    p2 = follower(['go', '--from', 'home', 'b', '--confirm'], env)
    o2, _ = p2.communicate(timeout=120)
    time.sleep(3.0)
    p1.send_signal(signal.SIGINT); o1, _ = p1.communicate(timeout=60)
    ran = [o for o in (o1, o2) if '[preflight] OK' in o]
    refused = [o for o in (o1, o2) if 'another motion tool is running' in o]
    bad = [] if (len(ran) == 1 and len(refused) == 1) else [f'{len(ran)} ran, {len(refused)} refused the lock']
    report('two followers 0.05 s apart', bad, refused[0] if refused else o2)
    settle_check([])

    # the map file changes after the waypoints were captured: must refuse
    with open(mapf, 'w') as f:
        f.write('# fake map v2 -- rebuilt\n')
    robot.reset(Sim(seed=5))
    time.sleep(0.5)
    p = follower(['go', '--from', 'home', 'a', '--confirm'], env)
    out, _ = p.communicate(timeout=60)
    bad = [] if 'has changed since these waypoints were captured' in out else ['did not refuse a changed map']
    with robot.lock:
        if robot.pnc:
            bad.append(f'{len(robot.pnc)} commands published')
    report('go: refused, map changed', bad, out)
    with open(mapf, 'w') as f:
        f.write('# fake map v1\n')

    # shadow: the operator drives (scripted on the rc channel); the follower has no publisher
    route = [('a', 3.0, 0.0), ('b', 3.0, 2.0)]
    goals = {'i': 0}

    def driver(s):
        if goals['i'] >= len(route):
            return (0.0, 0.0)
        _, gx, gy = route[goals['i']]
        dx, dy = gx - s.x, gy - s.y
        if math.hypot(dx, dy) < 0.12:
            goals['i'] += 1
            return (0.0, 0.0)
        e = fc.wrap(math.atan2(dy, dx) - s.yaw)
        if abs(e) > 0.3:
            return (0.0, math.copysign(0.7, e))           # in place: above mc's ~0.5 threshold
        return (0.25, max(-0.5, min(0.5, 1.2 * e)))
    robot.reset(Sim(seed=6, shadow_driver=driver))
    time.sleep(0.5)
    p = follower(['shadow', 'a', 'b'], env)
    out, _ = p.communicate(timeout=180)
    bad = []
    with robot.lock:
        if robot.pnc:
            bad.append(f'shadow published {len(robot.pnc)} commands')
    if '[result] COMPLETED' not in out:
        bad.append('shadow did not complete the route')
    if 'STILL MOVING' in out:
        bad.append('false "STILL MOVING" alarm')
    report('shadow a b (operator drives)', bad, out)

    print(f'\n{"ALL PASSED" if not fails else f"{len(fails)} FAILED: {fails}"}   (logs in {tmp})')
    ex.shutdown()
    rclpy.shutdown()
    spin_t.join(timeout=2.0)
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
