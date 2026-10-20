#!/usr/bin/env python3
"""Live RGB and YOLO topic viewer for the recorded X11 screen; no robot commands."""
import sys

from cv_bridge import CvBridge
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = rclpy.create_node('video_live_camera_viewer')
    bridge = CvBridge()
    app = QtWidgets.QApplication(sys.argv[:1])
    window = QtWidgets.QWidget(flags=QtCore.Qt.FramelessWindowHint)
    window.setGeometry(0, 60, 320, 495)
    window.setStyleSheet('background:#172230;color:white;font:10pt "DejaVu Sans";')
    layout = QtWidgets.QVBoxLayout(window)
    layout.setContentsMargins(4, 2, 4, 2)
    labels = {}
    for title, topic in [('YOLO · /yolo/annotated_image', '/yolo/annotated_image'),
                         ('RGB · camera robot', '/oakd/rgb/preview/image_raw')]:
        layout.addWidget(QtWidgets.QLabel(title))
        label = QtWidgets.QLabel('Đang chờ ảnh từ ROS…')
        label.setAlignment(QtCore.Qt.AlignCenter)
        label.setMinimumSize(300, 200)
        layout.addWidget(label, 1)
        labels[topic] = label

    def display(topic, msg):
        pixels = bridge.imgmsg_to_cv2(msg, 'rgb8')
        h, w, _ = pixels.shape
        picture = QtGui.QImage(pixels.data, w, h, pixels.strides[0], QtGui.QImage.Format_RGB888).copy()
        label = labels[topic]
        label.setPixmap(QtGui.QPixmap.fromImage(picture).scaled(
            label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    subscriptions = [node.create_subscription(Image, topic, lambda msg, t=topic: display(t, msg),
                                               qos_profile_sensor_data) for topic in labels]
    timer = QtCore.QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    timer.start(10)
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
