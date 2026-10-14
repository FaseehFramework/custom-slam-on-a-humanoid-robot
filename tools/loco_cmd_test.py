#!/usr/bin/env python3
"""FIRST MOTION TEST: does publishing /aima/mc/locomotion/velocity make the X2 walk?

This is the only tool in the project that can make the robot MOVE. It runs ON soc1.
Every safety rule is enforced in code, not left to the procedure:

  INTERLOCK   refuses to publish unless GetMcAction reports action_desc ==
              'LOCOMOTION_DEFAULT' and status RUNNING. Uses the DESCRIPTION, because the
              numeric field reads 0 even when the description is set (measured 2026-10-14).
              Our code NEVER changes the motion mode -- the operator does, with the remote.
  HARD CAPS   |forward| <= 0.20 m/s, lateral = angular = 0, duration <= 3.0 s.
  ABORT       re-checks the mode at 5 Hz while publishing; any change (remote e-stop,
              mode switch, service failure) stops publishing at once.
  STOP        always ends with 1.0 s of explicit zero commands, including on Ctrl-C or
              any exception, then destroys the publisher.
  NO LATCH    BEST_EFFORT + VOLATILE, matching the mc subscriber; a TRANSIENT_LOCAL
              writer would hand our last command to any subscriber that (re)starts later.
  STOP CHECK  watches leg odometry for 4 s after the zeros; if the robot is still moving,
              prints a loud warning (then the remote e-stop is the answer, and a timeout
              watchdog becomes a requirement before any further work).

It logs commands seen on the topic (ALL sources, so arbitration with the remote's 49 Hz
zero stream is visible) and leg-odometry world motion to a CSV, and prints a verdict.

Usage on soc1 (dry run first: does everything except publish a non-zero command):
  python3 loco_cmd_test.py --dry-run
  python3 loco_cmd_test.py --forward 0.10 --duration 2.0
"""
import argparse, csv, math, os, signal, sys, threading, time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from aimdk_msgs.msg import McLocomotionVelocity
from aimdk_msgs.srv import GetMcAction
from nav_msgs.msg import Odometry

TOPIC = '/aima/mc/locomotion/velocity'
ACTION_SRV = '/aimdk_5Fmsgs/srv/GetMcAction'
REQUIRED_ACTION = 'LOCOMOTION_DEFAULT'
RUNNING = 100
MAX_FWD, MAX_DUR, RATE_HZ, ZERO_TAIL_S, WATCH_AFTER_S = 0.20, 3.0, 50.0, 1.0, 4.0
SOURCE = 'x2slam_test'   # our own tag: also tests whether mc filters by source


class Tester(Node):
    def __init__(self, log_path):
        super().__init__('x2slam_loco_test')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(McLocomotionVelocity, TOPIC, qos)
        self.cli = self.create_client(GetMcAction, ACTION_SRV)
        self.create_subscription(McLocomotionVelocity, TOPIC, self.on_cmd, qos)
        self.create_subscription(Odometry, '/aima/mc/leg_odometry', self.on_leg, qos)
        self.lock = threading.Lock()
        self.log = open(log_path, 'w', newline='')
        self.w = csv.writer(self.log)
        self.w.writerow(['t', 'kind', 'source', 'fwd', 'lat', 'ang', 'x', 'y', 'z'])
        self.seq = 0
        self.last_leg = None

    def on_cmd(self, m):
        with self.lock:
            self.w.writerow([f'{time.time():.4f}', 'cmd', m.source, m.forward_velocity,
                             m.lateral_velocity, m.angular_velocity, '', '', ''])

    def on_leg(self, m):
        p = m.pose.pose.position
        with self.lock:
            self.last_leg = (time.time(), p.x, p.y, p.z)
            self.w.writerow([f'{time.time():.4f}', 'leg', '', '', '', '', p.x, p.y, p.z])

    def action(self):
        """(ok, description) -- ok only for LOCOMOTION_DEFAULT + RUNNING."""
        if not self.cli.service_is_ready() and not self.cli.wait_for_service(timeout_sec=3.0):
            return False, 'action service unavailable'
        fut = self.cli.call_async(GetMcAction.Request())
        t_end = time.time() + 2.0
        while not fut.done() and time.time() < t_end:
            time.sleep(0.01)
        r = fut.result() if fut.done() else None
        if r is None:
            return False, 'no response from action service'
        desc, st = r.info.action_desc, r.info.status.value
        return (desc == REQUIRED_ACTION and st == RUNNING), f'{desc} status={st}'

    def send(self, fwd):
        m = McLocomotionVelocity()
        now = self.get_clock().now().to_msg()
        m.header.stamp = now
        m.header.meas_stamp = now
        m.header.frame_id = 'locomotion_velocity'
        m.header.sequence = self.seq
        self.seq += 1
        m.source = SOURCE
        m.forward_velocity = float(fwd)
        m.lateral_velocity = 0.0
        m.angular_velocity = 0.0
        self.pub.publish(m)
        with self.lock:
            self.w.writerow([f'{time.time():.4f}', 'sent', SOURCE, fwd, 0.0, 0.0, '', '', ''])


def horiz_speed(samples):
    if len(samples) < 2:
        return None
    (t0, x0, y0, _), (t1, x1, y1, _) = samples[0], samples[-1]
    return math.hypot(x1 - x0, y1 - y0) / (t1 - t0) if t1 > t0 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--forward', type=float, default=0.10)
    ap.add_argument('--duration', type=float, default=2.0)
    ap.add_argument('--dry-run', action='store_true',
                    help='run every check and the logging, but publish only zeros')
    ap.add_argument('--log', default=os.path.expanduser('~/slam_ws/loco_test.csv'))
    a = ap.parse_args()
    if not (0.0 <= a.forward <= MAX_FWD):
        sys.exit(f'--forward must be within 0..{MAX_FWD} m/s (hard cap)')
    if not (0.0 < a.duration <= MAX_DUR):
        sys.exit(f'--duration must be within 0..{MAX_DUR} s (hard cap)')
    fwd = 0.0 if a.dry_run else a.forward

    rclpy.init()
    n = Tester(a.log)
    spin = threading.Thread(target=rclpy.spin, args=(n,), daemon=True)
    spin.start()
    try:
        return run(n, a, fwd)
    finally:
        # clean shutdown on EVERY path (the refusal path used to exit with ROS still
        # spinning, which aborted the interpreter with "terminate called")
        n.log.close()
        rclpy.shutdown()
        spin.join(timeout=2.0)


def run(n, a, fwd):
    period = 1.0 / RATE_HZ
    stopping = {'flag': False}
    entered = {'loop': False}    # zeros are sent ONLY if we actually started commanding

    def zeros(seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            n.send(0.0)
            time.sleep(period)

    def emergency(*_):
        stopping['flag'] = True
    signal.signal(signal.SIGINT, emergency)
    signal.signal(signal.SIGTERM, emergency)

    # ---- interlock: nothing is published before this passes -------------------------
    ok, desc = n.action()
    print(f'[interlock] motion mode: {desc}')
    if not ok:
        print(f'[interlock] REFUSING: need {REQUIRED_ACTION} + RUNNING. Nothing was published. '
              f'Put the robot in walk mode with the remote first.')
        return 3
    time.sleep(1.0)                           # baseline: let leg odometry and cmd logging settle
    with n.lock:
        base = n.last_leg
    if base is None:
        print('[check] no leg odometry received -- refusing (cannot verify motion or stop). '
              'Nothing was published.')
        return 4

    verdict = 'ABORTED'
    legs_during = []
    try:
        print(f'[test] publishing forward={fwd:.2f} m/s for {a.duration:.1f} s '
              f'({"DRY RUN: zeros only" if a.dry_run else "LIVE"}); Ctrl-C aborts to zeros')
        entered['loop'] = True
        t0 = time.time(); next_check = t0
        while time.time() - t0 < a.duration and not stopping['flag']:
            if time.time() >= next_check:
                ok, desc = n.action()
                if not ok:
                    print(f'[abort] motion mode changed to: {desc}')
                    break
                next_check = time.time() + 0.2
            n.send(fwd)
            with n.lock:
                if n.last_leg: legs_during.append(n.last_leg)
            time.sleep(period)
        else:
            verdict = 'COMPLETED' if not stopping['flag'] else 'ABORTED (Ctrl-C)'
    finally:
        if entered['loop']:
            zeros(ZERO_TAIL_S)                # explicit stop, whatever happened above
            print('[stop] sent zeros')
        n.destroy_publisher(n.pub)

    # ---- did it stop? watch leg odometry after our last message ----------------------
    after = []
    t_end = time.time() + WATCH_AFTER_S
    while time.time() < t_end:
        with n.lock:
            if n.last_leg: after.append(n.last_leg)
        time.sleep(0.05)
    v_during = horiz_speed(legs_during[len(legs_during)//2:]) if legs_during else None
    v_late = horiz_speed(after[len(after)//2:])
    moved = math.hypot(after[-1][1] - base[1], after[-1][2] - base[2]) if after else None
    r3 = lambda v: None if v is None else round(v, 3)
    print(f'\n[result] {verdict}')
    print(f'  speed in 2nd half of command window : {r3(v_during)} m/s (commanded {fwd:.2f})')
    print(f'  speed 2-4 s after zeros            : {r3(v_late)} m/s')
    print(f'  net displacement since start       : {r3(moved)} m')
    if v_late is not None and v_late > 0.05:
        print('\n  !!! ROBOT STILL MOVING AFTER ZEROS -- use the remote e-stop. A command timeout '
              'watchdog is REQUIRED before any further motion work. !!!')
    print(f'  log: {a.log}')
    return 0

if __name__ == '__main__':
    sys.exit(main())
