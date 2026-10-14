#!/usr/bin/env python3
"""Decode /aima/mc/locomotion/velocity (aimdk_msgs/msg/McLocomotionVelocity) from a bag
without the aimdk type support (it is aarch64-only), by parsing the CDR directly.

Definition (from `ros2 interface show` on soc1, 2026-10-14):
  MessageHeader header { Time stamp; string frame_id; uint32 sequence; Time meas_stamp }
  string  source
  float64 forward_velocity   m/s, + forward
  float64 lateral_velocity   m/s, + left
  float64 angular_velocity   + turn left (vendor comment says m/s; almost certainly rad/s)

Usage: tools/decode_locomotion_cmd.py data/bags/<bag>
"""
import sys, struct, math, bisect, collections
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

TOPIC = '/aima/mc/locomotion/velocity'


class CDR:
    def __init__(self, raw):
        self.b, self.p = raw, 4            # skip the 4-byte encapsulation header
    def align(self, n):
        off = self.p - 4
        self.p += (-off) % n
    def u32(self):
        self.align(4); v = struct.unpack_from('<I', self.b, self.p)[0]; self.p += 4; return v
    def i32(self):
        self.align(4); v = struct.unpack_from('<i', self.b, self.p)[0]; self.p += 4; return v
    def f64(self):
        self.align(8); v = struct.unpack_from('<d', self.b, self.p)[0]; self.p += 8; return v
    def string(self):
        n = self.u32(); s = self.b[self.p:self.p + n - 1].decode('utf-8', 'replace'); self.p += n; return s


def parse(raw):
    c = CDR(raw)
    sec, nsec = c.i32(), c.u32()
    frame = c.string()
    seq = c.u32()
    msec, mnsec = c.i32(), c.u32()
    src = c.string()
    return {'t': sec + nsec * 1e-9, 'frame': frame, 'seq': seq, 'source': src,
            'fwd': c.f64(), 'lat': c.f64(), 'ang': c.f64()}


def read(bag, topic):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    while r.has_next():
        yield r.read_next()


def main():
    bag = sys.argv[1]
    msgs = []
    for _, raw, recv in read(bag, TOPIC):
        m = parse(raw); m['recv'] = recv * 1e-9; msgs.append(m)
    if not msgs:
        sys.exit(f'no {TOPIC} in {bag}')
    dur = msgs[-1]['recv'] - msgs[0]['recv']
    print(f'{len(msgs)} msgs over {dur:.1f} s ({len(msgs)/dur:.1f} Hz)')
    by = collections.defaultdict(list)
    for m in msgs:
        by[m['source']].append(m)
    print(f"\n  {'source':<28} {'n':>6} {'Hz':>6} {'nonzero':>8}  fwd range        lat range        ang range")
    for s, ms in sorted(by.items(), key=lambda x: -len(x[1])):
        nz = sum(1 for m in ms if abs(m['fwd']) + abs(m['lat']) + abs(m['ang']) > 1e-9)
        rng = lambda k: f"[{min(m[k] for m in ms):+.3f},{max(m[k] for m in ms):+.3f}]"
        print(f"  {s!r:<28} {len(ms):>6} {len(ms)/dur:>6.1f} {nz:>8}  {rng('fwd'):<16} {rng('lat'):<16} {rng('ang')}")
    print(f"  frame_ids seen: {sorted({m['frame'] for m in msgs})}")
    # do commands track actual motion? Leg odometry's BODY frame is the chest IMU frame,
    # pitched ~-85 deg, so its twist.x is NOT robot-forward. Use the WORLD-frame pose:
    # horizontal speed from position differences, heading rate from the world-delta rotation.
    sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
    from eval_loop_closure import load_full, rotmat, rel_rot
    lT, lP, lQ, _ = load_full(bag, '/aima/mc/leg_odometry')
    if lT:
        def motion(t, w=0.5):
            a = bisect.bisect_left(lT, t - w / 2); b = min(bisect.bisect_left(lT, t + w / 2), len(lT) - 1)
            if b <= a: return None, None
            dt = lT[b] - lT[a]
            v = math.hypot(lP[b][0] - lP[a][0], lP[b][1] - lP[a][1]) / dt
            head, _, _, _ = rel_rot(rotmat(lQ[a]), rotmat(lQ[b]))
            return v, math.radians(head) / dt
        print(f"\n  command vs leg-odometry WORLD motion (0.5 s window; 1 s bins where a command is nonzero):")
        print(f"  {'t':>6} {'src':<8} {'cmd fwd':>8} {'cmd lat':>8} {'speed':>7} {'cmd ang':>8} {'yaw rate':>9}")
        t0 = msgs[0]['t']; last = -9
        rows = []
        for m in msgs:
            if abs(m['fwd']) + abs(m['lat']) + abs(m['ang']) < 1e-6 or m['t'] - last < 1.0:
                continue
            last = m['t']
            v, wz = motion(m['t'])
            if v is None: continue
            rows.append((m, v, wz))
            print(f"  {m['t']-t0:>6.1f} {m['source'][:8]:<8} {m['fwd']:>+8.3f} {m['lat']:>+8.3f} {v:>7.3f} "
                  f"{m['ang']:>+8.3f} {wz:>+9.3f}")
        if rows:
            import numpy as np
            cf = np.array([r[0]['fwd'] for r in rows]); sp = np.array([r[1] for r in rows])
            ca = np.array([r[0]['ang'] for r in rows]); yr = np.array([r[2] for r in rows])
            print(f"\n  corr(cmd fwd, speed) {np.corrcoef(cf, sp)[0,1]:+.2f}   "
                  f"corr(cmd ang, yaw rate) {np.corrcoef(ca, yr)[0,1]:+.2f}   (n={len(rows)})")
            print(f"  max speed achieved {sp.max():.3f} m/s while max cmd fwd {cf.max():.3f}")

if __name__ == '__main__':
    main()
