#!/usr/bin/env python3
"""Offline, deterministic conversion of an X2 Ultra bag into Point-LIO-ready form.

Does exactly what x2_cloud_adapter does, but as a file->file transform with no DDS
in the loop, so the result is byte-identical every run:

  vendor  : x,y,z(f32) intensity(f32@16) ring(u16@20) timestamp(f64@24)  step 32
  velodyne: x,y,z(f32) intensity(f32@16) time(f32@20)  ring(u16@24)      step 32

`time` becomes float32 SECONDS (vendor stores float64 nanoseconds). All other topics
are copied through untouched.

  ./tools/convert_bag.py data/bags/walk_20261007_102515 data/bags/walk_..._plio
"""
import sys, os
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import PointCloud2, PointField

VENDOR_CLOUD = '/aima/hal/sensor/lidar_chest_front/lidar_pointcloud'
OUT_CLOUD    = '/x2/points'
TIME_SCALE   = 1.0e-9          # vendor ns -> seconds

IN_DT = np.dtype({'names':   ['x','y','z','_pad','intensity','ring','_pad2','timestamp'],
                  'formats': ['<f4','<f4','<f4','<f4','<f4','<u2','<u2','<f8'],
                  'offsets': [0,4,8,12,16,20,22,24], 'itemsize':32})
OUT_DT = np.dtype({'names':   ['x','y','z','_pad','intensity','time','ring','_pad2'],
                   'formats': ['<f4','<f4','<f4','<f4','<f4','<f4','<u2','<u2'],
                   'offsets': [0,4,8,12,16,20,24,26], 'itemsize':32})

def out_fields():
    f = lambda n,o,d: PointField(name=n, offset=o, datatype=d, count=1)
    return [f('x',0,PointField.FLOAT32), f('y',4,PointField.FLOAT32),
            f('z',8,PointField.FLOAT32), f('intensity',16,PointField.FLOAT32),
            f('time',20,PointField.FLOAT32), f('ring',24,PointField.UINT16)]

def main(src, dst):
    if os.path.exists(dst):
        sys.exit(f"refusing to overwrite existing {dst}")
    rd = rosbag2_py.SequentialReader()
    rd.open(rosbag2_py.StorageOptions(uri=src, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr','cdr'))
    topics = {t.name: t for t in rd.get_all_topics_and_types()}
    if VENDOR_CLOUD not in topics:
        sys.exit(f"{src} has no {VENDOR_CLOUD}")

    wr = rosbag2_py.SequentialWriter()
    wr.open(rosbag2_py.StorageOptions(uri=dst, storage_id='sqlite3'),
            rosbag2_py.ConverterOptions('cdr','cdr'))
    for name, meta in topics.items():
        if name == VENDOR_CLOUD:
            wr.create_topic(rosbag2_py.TopicMetadata(
                name=OUT_CLOUD, type=meta.type, serialization_format='cdr'))
        else:
            wr.create_topic(rosbag2_py.TopicMetadata(
                name=name, type=meta.type, serialization_format='cdr'))

    nclouds = nother = 0
    while rd.has_next():
        topic, raw, t = rd.read_next()
        if topic != VENDOR_CLOUD:
            wr.write(topic, raw, t); nother += 1; continue
        m = deserialize_message(raw, PointCloud2)
        n = m.width * m.height
        a = np.frombuffer(m.data, dtype=IN_DT, count=n)
        o = np.zeros(n, dtype=OUT_DT)
        o['x'] = a['x']; o['y'] = a['y']; o['z'] = a['z']
        o['intensity'] = a['intensity']
        o['time'] = (a['timestamp'] * TIME_SCALE).astype('<f4')
        o['ring'] = a['ring']
        out = PointCloud2()
        out.header = m.header
        out.height, out.width = 1, n
        out.fields = out_fields()
        out.is_bigendian = m.is_bigendian
        out.point_step = 32
        out.row_step = 32 * n
        out.is_dense = m.is_dense
        out.data = o.tobytes()
        wr.write(OUT_CLOUD, serialize_message(out), t)
        nclouds += 1
    del wr
    print(f"converted {nclouds} clouds, copied {nother} other messages -> {dst}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2])
