#!/usr/bin/env python3
"""Loop-closure displacement expressed as FORWARD / LEFT of the robot's start pose.

The operator reports repositioning as "behind", "to the right", ... so the estimator's
closure vector has to be put into those terms to be compared.

Robot-forward in the body (IMU) frame is NOT an axis of that frame: the IMU is pitched
~-85 deg. It is measured from the data instead: the mean direction of leg odometry's
body-frame velocity while walking faster than 0.3 m/s (the robot walks forward almost all
the time). Then forward_world = horizontal part of R_start * f_body, and left = z x forward.

Usage: tools/closure_in_robot_frame.py <result_bag> <data_bag>
"""
import sys, os, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from eval_loop_closure import load_full, dwells, mean_pose
from dr_payoff import EST_TOPIC, REF_TOPIC
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def body_forward(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    r.set_filter(rosbag2_py.StorageFilter(topics=[REF_TOPIC]))
    acc, n = np.zeros(3), 0
    while r.has_next():
        _, raw, _ = r.read_next()
        v = deserialize_message(raw, Odometry).twist.twist.linear
        v = np.array([v.x, v.y, v.z]); s = np.linalg.norm(v)
        if s > 0.3:
            acc += v / s; n += 1
    f = acc / np.linalg.norm(acc)
    return f, n, np.linalg.norm(acc) / n      # consistency: 1.0 = always the same direction


def main():
    res, bag = sys.argv[1], sys.argv[2]
    f_body, n, cons = body_forward(bag)
    rT, _, _, rS = load_full(bag, REF_TOPIC)
    d = dwells(rT, rS, 0.05, 3.0)
    eT, eP, eQ, _ = load_full(res, EST_TOPIC)
    (a0, a1), (b0, b1) = d[0], d[-1]
    p0, M0 = mean_pose(eT, eP, eQ, a0, a1)     # returns (position, rotation)
    p1, M1 = mean_pose(eT, eP, eQ, b0, b1)
    p0, p1 = np.array(p0), np.array(p1)
    fw = M0 @ f_body; fw[2] = 0; fw /= np.linalg.norm(fw)
    left = np.cross([0, 0, 1.0], fw)
    dp = p1 - p0
    print(f'robot-forward in body frame {np.round(f_body, 3)} (from {n} walking samples, consistency {cons:.2f})')
    print(f'closure {np.linalg.norm(dp[:2]):.3f} m horizontal:  forward {dp @ fw:+.3f} m   left {dp @ left:+.3f} m'
          f'   (negative forward = behind, negative left = right)   dz {dp[2]:+.3f} m')


if __name__ == '__main__':
    main()
