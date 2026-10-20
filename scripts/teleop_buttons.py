#!/usr/bin/env python3
"""Hold-to-drive buttons for manual mapping; publishes ordinary ROS Twist."""
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from PyQt5.QtCore import QEvent, QTimer
from PyQt5.QtWidgets import (
    QApplication, QDoubleSpinBox, QGridLayout, QLabel, QPushButton, QWidget,
)


class TeleopWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        self.motion = None
        self.stop_until = 0.0
        self.last_odom = 0.0
        self.position = None
        node.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.setWindowTitle('TurtleBot 4 — Nút điều khiển mapping')
        self.resize(510, 360)
        self.move(80, 100)
        self.setStyleSheet('QPushButton { font-size: 20px; min-height: 66px; } '
                          'QLabel, QDoubleSpinBox { font-size: 16px; }')
        layout = QGridLayout(self)
        layout.addWidget(QLabel('Nhấn giữ nút để chạy. Thả nút để dừng.'), 0, 0, 1, 3)
        self.speed = QDoubleSpinBox()
        self.speed.setRange(0.05, 0.20)
        self.speed.setSingleStep(0.05)
        self.speed.setValue(0.15)
        self.speed.setSuffix(' m/s')
        layout.addWidget(QLabel('Tốc độ:'), 1, 0)
        layout.addWidget(self.speed, 1, 1)
        for text, row, col, linear, angular in [
            ('↑ Tiến', 2, 1, 1, 0), ('↶ Trái', 3, 0, 0, 0.35),
            ('↷ Phải', 3, 2, 0, -0.35), ('↓ Lùi', 4, 1, -1, 0),
        ]:
            button = QPushButton(text)
            button.pressed.connect(lambda v=linear, w=angular: self.drive(v, w))
            button.released.connect(self.stop)
            layout.addWidget(button, row, col)
        stop_button = QPushButton('DỪNG')
        stop_button.setStyleSheet('background: #c62828; color: white; font-weight: bold;')
        stop_button.clicked.connect(self.stop)
        layout.addWidget(stop_button, 3, 1)
        self.status = QLabel('Đang chờ dữ liệu robot…')
        layout.addWidget(self.status, 5, 0, 1, 3)
        layout.addWidget(QLabel('Đóng bảng điều khiển trước khi chạy Nav2 Goal.'), 6, 0, 1, 3)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

    def on_odom(self, msg):
        self.last_odom = time.monotonic()
        self.position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def drive(self, linear, angular):
        if time.monotonic() - self.last_odom < 3.0:
            self.motion = (linear * self.speed.value(), angular)
            self.publish(*self.motion)

    def publish(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher.publish(msg)

    def stop(self):
        self.motion = None
        self.stop_until = time.monotonic() + 0.3
        self.publish()

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0)
        now = time.monotonic()
        if now - self.last_odom >= 3.0:
            if self.motion is not None:
                self.stop()
            self.status.setText('Chưa nhận odometry — kiểm tra Gazebo đang chạy.')
        else:
            x, y = self.position
            self.status.setText(f'Robot đang chạy dữ liệu: x={x:.2f}, y={y:.2f} m')
        if self.motion is not None:
            self.publish(*self.motion)
        elif now < self.stop_until:
            self.publish()

    def event(self, event):
        if event.type() == QEvent.WindowDeactivate and hasattr(self, 'publisher'):
            self.stop()
        return super().event(event)

    def closeEvent(self, event):
        self.timer.stop()
        self.stop()
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.03)
            self.publish()
        event.accept()


def main():
    rclpy.init()
    node = rclpy.create_node('teleop_buttons')
    app = QApplication(sys.argv)
    window = TeleopWindow(node)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
