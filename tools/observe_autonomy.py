#!/usr/bin/env python3
"""Read-only telemetry/real camera capture. Never sends goals, poses or velocities.

Use a fresh output directory. Stops after --seconds or Ctrl+C. Saved annotated
pixels come from /yolo/annotated_image, not a generated illustration.
"""
import argparse
from collections import OrderedDict
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.msg import HazardDetectionVector
from nav_msgs.msg import Odometry, Path as NavPath
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=400)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('autonomy_observer')
    bridge = CvBridge()
    data = {'method': 'Read-only ROS telemetry and real RGB/annotated camera frames',
            'amcl_trajectory': [], 'hazards': [], 'detections': [], 'rgb_frames': 0,
            'annotated_frames': 0, 'detection_frames': 0, 'plan_messages': 0, 'gazebo_trajectory': []}
    frames = OrderedDict()
    pending = OrderedDict()
    best = [0.0]
    last_gt = [0.0]

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def rgb(msg):
        data['rgb_frames'] += 1
        if data['rgb_frames'] == 1:
            cv2.imwrite(str(args.output / 'camera_rgb.png'), bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def annotated(msg):
        data['annotated_frames'] += 1
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        frames[key] = msg
        if key in pending:
            save_detection(pending[key], msg)
        while len(frames) > 20:
            frames.popitem(last=False)
        if data['annotated_frames'] == 1:
            cv2.imwrite(str(args.output / 'yolo_camera.png'), bridge.imgmsg_to_cv2(msg, 'bgr8'))

    def detections(msg):
        data['detection_frames'] += 1
        value = json.loads(msg.data)
        if value['objects']:
            data['detections'].append(value)
            t = value['header']['stamp']
            key = (t['sec'], t['nanosec'])
            pending[key] = value
            while len(pending) > 20:
                pending.popitem(last=False)
            if key in frames:
                save_detection(value, frames[key])

    def save_detection(value, image):
        score = max(o['confidence'] for o in value['objects'])
        if score > best[0]:
            cv2.imwrite(str(args.output / 'yolo_camera.png'), bridge.imgmsg_to_cv2(image, 'bgr8'))
            best[0] = score
            (args.output / 'yolo_camera.json').write_text(json.dumps(value, indent=2))

    def pose(msg):
        p = msg.pose.pose.position
        data['amcl_trajectory'].append([stamp(msg), p.x, p.y])

    def ground_truth(msg):
        t = stamp(msg)
        if t - last_gt[0] > 0.5:
            p = msg.pose.pose.position
            data['gazebo_trajectory'].append([t, p.x, p.y])
            last_gt[0] = t

    def hazard(msg):
        if msg.detections:
            data['hazards'].append({'sim_time': stamp(msg), 'types': [h.type for h in msg.detections]})

    def plan(_):
        data['plan_messages'] += 1

    subscriptions = [node.create_subscription(kind, topic, callback, qos_profile_sensor_data)
                     for kind, topic, callback in [
                         (Image, '/oakd/rgb/preview/image_raw', rgb),
                         (Image, '/yolo/annotated_image', annotated),
                         (String, '/yolo/detections', detections),
                         (PoseWithCovarianceStamped, '/amcl_pose', pose),
                         (Odometry, '/sim_ground_truth_pose', ground_truth),
                         (HazardDetectionVector, '/hazard_detection', hazard),
                         (NavPath, '/plan', plan)]]
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        data['elapsed_wall_s'] = round(time.monotonic() - started, 3)
        (args.output / 'telemetry.json').write_text(json.dumps(data, indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(args.output)


if __name__ == '__main__':
    main()
