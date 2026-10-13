#!/usr/bin/env python3
"""Sanity-check a recorded X2 Ultra dataset: continuity, dynamics, trajectory, frames."""
import sys, math, struct, statistics as st
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

BAG = sys.argv[1]
WANT = {
 '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud': PointCloud2,
 '/aima/hal/sensor/lidar_chest_front/imu': Imu,
 '/aima/hal/imu/torso/state': Imu,
 '/aima/mc/leg_odometry': Odometry,
 '/tf_static': TFMessage,
}
def rdr():
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr','cdr'))
    return r

data = {k: [] for k in WANT}
r = rdr(); r.set_filter(rosbag2_py.StorageFilter(topics=list(WANT)))
while r.has_next():
    t, raw, _ = r.read_next()
    if t in WANT: data[t].append(deserialize_message(raw, WANT[t]))

def hs(m): return m.header.stamp.sec + m.header.stamp.nanosec*1e-9

# ---------------------------------------------------------------- point clouds
cl = data['/aima/hal/sensor/lidar_chest_front/lidar_pointcloud']
ts = [hs(m) for m in cl]
dts = [ts[i]-ts[i-1] for i in range(1,len(ts))]
nominal = st.median(dts)
gaps = [(i, d) for i, d in enumerate(dts) if d > nominal*1.5]
widths = [m.width for m in cl]
print("=== POINT CLOUDS ===")
print(f"  count {len(cl)}  span {ts[-1]-ts[0]:.2f}s  median dt {nominal*1e3:.2f} ms ({1/nominal:.2f} Hz)")
print(f"  dt    min {min(dts)*1e3:.2f}  max {max(dts)*1e3:.2f}  sd {st.pstdev(dts)*1e3:.2f} ms")
print(f"  DROPPED SCANS (dt > 1.5x nominal): {len(gaps)}")
for i,d in gaps[:5]: print(f"      at scan {i}: {d*1e3:.1f} ms")
print(f"  width min {min(widths)}  max {max(widths)}  mean {st.mean(widths):.0f}")

# per-point timestamp sanity on a mid-walk cloud
m = cl[len(cl)//2]; ps = m.point_step
pt = [struct.unpack_from('<d', m.data, i*ps+24)[0] for i in range(0, m.width, max(1,m.width//3000))]
print(f"  per-point ts span {(max(pt)-min(pt))*1e-6:.2f} ms  (expect ~{nominal*1e3:.0f} ms)")

# ---------------------------------------------------------------- imu
def imustats(name, msgs):
    a=[math.sqrt(m.linear_acceleration.x**2+m.linear_acceleration.y**2+m.linear_acceleration.z**2) for m in msgs]
    g=[math.sqrt(m.angular_velocity.x**2+m.angular_velocity.y**2+m.angular_velocity.z**2) for m in msgs]
    d=[abs(a[i]-a[i-1]) for i in range(1,len(a))]
    print(f"  {name:<22} |a| mean {st.mean(a):6.3f} sd {st.pstdev(a):6.3f} min {min(a):6.3f} max {max(a):7.3f} | "
          f"|w| max {max(g):5.3f} | d|a| p99 {sorted(d)[int(len(d)*.99)]:5.3f} max {max(d):6.3f}")
print("\n=== IMU ===")
imustats("lidar_chest_front", data['/aima/hal/sensor/lidar_chest_front/imu'])
imustats("torso", data['/aima/hal/imu/torso/state'])

# ---------------------------------------------------------------- odometry
od = data['/aima/mc/leg_odometry']
P = [(m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z) for m in od]
path = sum(math.dist(P[i],P[i-1]) for i in range(1,len(P)))
net  = math.dist(P[0],P[-1])
spd  = [math.sqrt(m.twist.twist.linear.x**2+m.twist.twist.linear.y**2) for m in od]
moving = sum(1 for v in spd if v > 0.05)
print("\n=== LEG ODOMETRY ===")
print(f"  frame '{od[0].header.frame_id}' -> child '{od[0].child_frame_id}'")
print(f"  path length {path:.2f} m   net displacement {net:.2f} m")
print(f"  start ({P[0][0]:+.2f},{P[0][1]:+.2f},{P[0][2]:+.2f})  end ({P[-1][0]:+.2f},{P[-1][1]:+.2f},{P[-1][2]:+.2f})")
print(f"  speed mean {st.mean(spd):.3f}  max {max(spd):.3f} m/s   moving {100.0*moving/len(spd):.1f}% of samples")

# ---------------------------------------------------------------- frames
print("\n=== TF_STATIC FRAME TREE ===")
seen=set()
for msg in data['/tf_static']:
    for tr in msg.transforms:
        k=(tr.header.frame_id, tr.child_frame_id)
        if k in seen: continue
        seen.add(k)
        t=tr.transform.translation
        print(f"  {tr.header.frame_id:<26} -> {tr.child_frame_id:<26} t=({t.x:+.4f},{t.y:+.4f},{t.z:+.4f})")

# ---------------------------------------------------------------- sync
print("\n=== TIME ALIGNMENT ===")
it=[hs(m) for m in data['/aima/hal/sensor/lidar_chest_front/imu']]
ot=[hs(m) for m in od]
print(f"  cloud  {ts[0]:.3f} .. {ts[-1]:.3f}")
print(f"  imu    {it[0]:.3f} .. {it[-1]:.3f}   (offset to cloud start {it[0]-ts[0]:+.3f}s)")
print(f"  odom   {ot[0]:.3f} .. {ot[-1]:.3f}   (offset to cloud start {ot[0]-ts[0]:+.3f}s)")
