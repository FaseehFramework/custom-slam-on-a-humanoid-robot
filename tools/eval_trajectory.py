#!/usr/bin/env python3
"""Compare a Point-LIO trajectory against the robot's leg odometry."""
import sys, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

def load(uri, topic):
    r=rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri,storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('cdr','cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    T=[];P=[]
    while r.has_next():
        _,raw,_=r.read_next(); m=deserialize_message(raw,Odometry)
        T.append(m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
        P.append((m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z))
    return T,P

est_t,est_p = load(sys.argv[1], '/aft_mapped_to_init')
ref_t,ref_p = load(sys.argv[2], '/aima/mc/leg_odometry')

def rel(P): 
    x0,y0,z0=P[0]; return [(x-x0,y-y0,z-z0) for x,y,z in P]
E=rel(est_p); R=rel(ref_p)
def plen(P): return sum(math.dist(P[i],P[i-1]) for i in range(1,len(P)))

print(f"=== Point-LIO  /aft_mapped_to_init ===")
print(f"  poses {len(E)}  span {est_t[-1]-est_t[0]:.1f}s  rate {len(E)/(est_t[-1]-est_t[0]):.1f} Hz")
print(f"  net displacement from start : {math.dist((0,0,0),E[-1]):.3f} m")
print(f"  final position              : ({E[-1][0]:+.3f}, {E[-1][1]:+.3f}, {E[-1][2]:+.3f})")
xs=[p[0] for p in E]; ys=[p[1] for p in E]; zs=[p[2] for p in E]
print(f"  extent x [{min(xs):+.2f},{max(xs):+.2f}]  y [{min(ys):+.2f},{max(ys):+.2f}]  z [{min(zs):+.3f},{max(zs):+.3f}]")

print(f"\n=== leg odometry (reference) ===")
print(f"  net displacement from start : {math.dist((0,0,0),R[-1]):.3f} m")
rxs=[p[0] for p in R]; rys=[p[1] for p in R]; rzs=[p[2] for p in R]
print(f"  extent x [{min(rxs):+.2f},{max(rxs):+.2f}]  y [{min(rys):+.2f},{max(rys):+.2f}]  z [{min(rzs):+.3f},{max(rzs):+.3f}]")

# the robot physically returned to ~0.15 m of start; use that as a drift probe
t0=ref_t[0]
best=min(((math.dist((0,0,0),R[i]), ref_t[i]-t0) for i in range(len(R)) if ref_t[i]-t0>30))
print(f"\n=== drift probe: robot physically back at start ===")
print(f"  reference says {best[0]:.3f} m from start at t={best[1]:.1f}s")
tt=est_t[0]+best[1]
j=min(range(len(est_t)), key=lambda i: abs(est_t[i]-tt))
d=math.dist((0,0,0),E[j])
print(f"  Point-LIO at that moment     : {d:.3f} m from its start")
print(f"  => horizontal/vertical split : xy {math.hypot(E[j][0],E[j][1]):.3f} m, z {E[j][2]:+.3f} m")
print(f"  => position drift over ~{best[1]:.0f}s of walking: ~{abs(d-best[0]):.3f} m")
print(f"\n  z drift at end (should be ~0, flat floor): {E[-1][2]:+.3f} m")
