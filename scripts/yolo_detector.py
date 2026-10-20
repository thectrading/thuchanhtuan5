#!/usr/bin/env python3
"""RGB Image -> cv_bridge/BGR -> YOLO -> annotated Image and JSON detections.

Inference runs in a dedicated thread with a single latest-frame slot. Slow CPU
inference cannot accumulate an unbounded queue of old camera frames.
"""
import json
import math
import os
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        defaults = {'image_topic': '/oakd/rgb/preview/image_raw',
                    'annotated_topic': '/yolo/annotated_image', 'detections_topic': '/yolo/detections',
                    'model': 'yolov8n.pt', 'confidence': 0.4, 'image_size': 416,
                    'device': 'cpu', 'max_fps': 3.0, 'show_window': False, 'log_interval': 2.0,
                    'inference_threads': 2}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.conf = self.value('confidence')
        self.fps = self.value('max_fps')
        if not math.isfinite(self.conf) or not 0 < self.conf <= 1:
            raise ValueError('confidence must be in (0, 1]')
        if not math.isfinite(self.fps) or self.fps <= 0 or self.value('image_size') <= 0:
            raise ValueError('max_fps and image_size must be positive')
        if not math.isfinite(self.value('log_interval')) or self.value('log_interval') <= 0:
            raise ValueError('log_interval must be positive')
        # Import here so missing ML dependencies produce an actionable error.
        os.environ.setdefault('OMP_WAIT_POLICY', 'PASSIVE')
        from ultralytics import YOLO
        import torch
        if self.value('inference_threads') < 1:
            raise ValueError('inference_threads must be positive')
        torch.set_num_threads(self.value('inference_threads'))
        self.torch = torch
        cv2.setNumThreads(1)
        self.model = YOLO(self.value('model'))
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, self.value('annotated_topic'),
                                         QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.detections = self.create_publisher(String, self.value('detections_topic'), 1)
        self.lock = threading.Lock()
        self.latest = None
        self.stopping = threading.Event()
        self.failed = False
        self.last_log = 0.0
        self.last_input = time.monotonic()
        self.sub = self.create_subscription(Image, self.value('image_topic'), self.receive,
                                            qos_profile_sensor_data)
        self.watchdog = self.create_timer(5.0, self.check_camera)
        self.worker = threading.Thread(target=self.infer, name='yolo_inference', daemon=True)
        self.worker.start()
        self.get_logger().info(f"YOLO {self.value('model')}: {self.value('image_topic')} -> {self.value('annotated_topic')}")

    def value(self, name):
        return self.get_parameter(name).value

    def receive(self, msg):
        with self.lock:
            self.latest = msg
            self.last_input = time.monotonic()

    def check_camera(self):
        if time.monotonic() - self.last_input > 5:
            self.get_logger().warning(f"No RGB frames: check {self.value('image_topic')} and camera bridge")

    def infer(self):
        try:
            while not self.stopping.is_set():
                started = time.monotonic()
                with self.lock:
                    msg, self.latest = self.latest, None
                if msg is None:
                    self.stopping.wait(0.02)
                    continue
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                # Ultralytics select_device resets CPU threads on first predict.
                # Reapply the cap after backend setup as well as on later frames.
                self.torch.set_num_threads(self.value('inference_threads'))
                result = self.model.predict(bgr, conf=self.conf, imgsz=self.value('image_size'),
                                            device=self.value('device'), verbose=False)[0]
                self.torch.set_num_threads(self.value('inference_threads'))
                annotated = result.plot()
                output = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                output.header = msg.header  # Preserve sensor timestamp and optical frame.
                self.pub.publish(output)
                objects = []
                if result.boxes is not None:
                    for box in result.boxes.cpu():
                        class_id = int(box.cls.item())
                        objects.append({'class_id': class_id, 'class_name': result.names[class_id],
                                        'confidence': round(float(box.conf.item()), 4),
                                        'xyxy': [round(float(v), 2) for v in box.xyxy[0].tolist()]})
                data = {'header': {'stamp': {'sec': msg.header.stamp.sec,
                                             'nanosec': msg.header.stamp.nanosec},
                                    'frame_id': msg.header.frame_id},
                        'image_width': msg.width, 'image_height': msg.height, 'objects': objects}
                self.detections.publish(String(data=json.dumps(data)))
                if started - self.last_log >= self.value('log_interval'):
                    summary = ', '.join(f"{o['class_name']} {o['confidence']:.0%}" for o in objects)
                    self.get_logger().info('Detected: ' + (summary or 'no objects above threshold'))
                    self.last_log = started
                if self.value('show_window'):
                    cv2.imshow('TurtleBot 4 YOLO', annotated)
                    cv2.waitKey(1)
                self.stopping.wait(max(0, 1 / self.fps - (time.monotonic() - started)))
        except Exception as exc:
            if not self.stopping.is_set():
                self.failed = True
                self.get_logger().error(f'YOLO inference stopped: {exc}')
        finally:
            if self.value('show_window'):
                cv2.destroyAllWindows()

    def close(self):
        self.stopping.set()
        self.worker.join()  # Finish the current inference before destroying publishers.


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = YoloDetector()
        while rclpy.ok() and not node.failed:
            rclpy.spin_once(node, timeout_sec=0.2)
        code = int(node.failed)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f'YOLO failed: {exc}. Use the Python environment described in README.md.', file=sys.stderr)
        code = 1
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
