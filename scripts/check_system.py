#!/usr/bin/env python3
"""Check live scan, odom, clock and TF without commanding robot motion."""
import argparse
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20)
    parser.add_argument('--map', action='store_true', help='Also require map -> base_link TF')
    args = parser.parse_args()
    rclpy.init()
    node = Node('lab_check', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    received = {}
    counts = {topic: 0 for topic in ('/scan', '/odom', '/clock')}
    def callback(topic):
        def record(message):
            received[topic] = message
            counts[topic] += 1
        return record
    subscriptions = [node.create_subscription(cls, topic, callback(topic), qos_profile_sensor_data)
                     for cls, topic in ((LaserScan, '/scan'), (Odometry, '/odom'), (Clock, '/clock'))]
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    frames = ['odom', 'map'] if args.map else ['odom']
    deadline = time.monotonic() + args.timeout
    ok = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        scan = received.get('/scan')
        ranges = [] if scan is None else [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        # A stream stuck entirely at the minimum range can pass a topic-presence test
        # while providing no usable geometry (a known renderer failure mode).
        valid_scan = bool(ranges) and max(ranges) > scan.range_min + 0.05
        ok = (valid_scan and all(n >= 2 for n in counts.values()) and
              all(buffer.can_transform(f, 'base_link', rclpy.time.Time()) for f in frames))
        if ok:
            break
    for topic, count in counts.items():
        print(f'{topic}: {count} messages')
    if received.get('/scan') is not None:
        print(f'LiDAR finite range span: {min(ranges) if ranges else None} .. {max(ranges) if ranges else None}; valid: {valid_scan}')
    for frame in frames:
        print(f'TF {frame} -> base_link: {buffer.can_transform(frame, "base_link", rclpy.time.Time())}')
    print('PASS' if ok else 'FAIL: missing/invalid sensor data or TF')
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
