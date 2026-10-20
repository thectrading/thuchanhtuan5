#!/usr/bin/env python3
"""Velocity-only mapping smoke test. This is NOT the assignment's keyboard teleop demo.

Use only in the default maze, with SLAM running and Nav2 stopped. Does not set poses,
generate occupancy maps, or inject ground truth. Stops on stale odom/scan or an obstacle.
"""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--route', required=True, help='JSON list of [x,y] waypoints in map')
    parser.add_argument('--timeout', type=float, default=600, help='Per-waypoint wall time limit')
    args = parser.parse_args()
    route = json.loads(Path(args.route).read_text())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('mapping_velocity_probe')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    state = {}
    def remember(key):
        def callback(msg): state[key] = (msg, time.monotonic())
        return callback
    odom_sub = node.create_subscription(Odometry, 'odom', remember('odom'), qos_profile_sensor_data)
    scan_sub = node.create_subscription(LaserScan, 'scan', remember('scan'), qos_profile_sensor_data)
    publisher = node.create_publisher(Twist, 'cmd_vel', 10)
    try:
        start = time.monotonic()
        while (len(state) < 2 or not buffer.can_transform('map','base_link',rclpy.time.Time())) and time.monotonic()-start < 20:
            rclpy.spin_once(node, timeout_sec=.1)
        if len(state) < 2: raise RuntimeError('No odom/scan')
        if any(name == 'bt_navigator' for name, _ in node.get_node_names_and_namespaces()):
            raise RuntimeError('Stop Nav2 before velocity-only mapping test.')
        for target in route:
            start = time.monotonic()
            progress_time = start
            best_distance = float('inf')
            print('TARGET', target, flush=True)
            while time.monotonic() - start < args.timeout:
                rclpy.spin_once(node, timeout_sec=.03)
                if any(time.monotonic()-state[k][1] > 5 for k in ('scan','odom')):
                    raise RuntimeError('Stale sensor data')
                transform = buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform
                p = transform.translation
                q = transform.rotation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                dx,dy = target[0]-p.x,target[1]-p.y
                distance=math.hypot(dx,dy)
                if distance < best_distance - .05:
                    best_distance = distance
                    progress_time = time.monotonic()
                if time.monotonic() - progress_time > 45:
                    raise RuntimeError('No progress for 45 seconds; inspect pose, hazards and route')
                if distance < .12:
                    publisher.publish(Twist())
                    print('REACHED', round(p.x,3),round(p.y,3),flush=True)
                    break
                error=math.atan2(math.sin(math.atan2(dy,dx)-yaw), math.cos(math.atan2(dy,dx)-yaw))
                msg=Twist();msg.angular.z=max(-.65,min(.65,1.5*error))
                if abs(error) < .2:
                    scan=state['scan'][0]
                    sensor = buffer.lookup_transform('base_link',scan.header.frame_id,rclpy.time.Time()).transform.rotation
                    sensor_yaw=math.atan2(2*(sensor.w*sensor.z+sensor.x*sensor.y),1-2*(sensor.y*sensor.y+sensor.z*sensor.z))
                    front=[r for i,r in enumerate(scan.ranges) if abs(math.atan2(math.sin(scan.angle_min+i*scan.angle_increment+sensor_yaw), math.cos(scan.angle_min+i*scan.angle_increment+sensor_yaw)))<.3 and math.isfinite(r)]
                    if front and min(front)<.4: raise RuntimeError(f'Obstacle ahead at {min(front):.3f} m')
                    msg.linear.x=min(.26,.6*distance)
                publisher.publish(msg)
            else: raise RuntimeError(f'Waypoint timeout: {target}')
        print('MAPPING PROBE COMPLETE', flush=True)
    finally:
        for _ in range(5): publisher.publish(Twist()); rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
