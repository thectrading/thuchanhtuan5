#!/usr/bin/env python3
"""Read-only video sidebar: display actual navigation JSON and node log files.

No ROS publisher, action client, robot command, or video replay is used here.
"""
import argparse
import json
from pathlib import Path
import re
import sys

from PyQt5 import QtCore, QtWidgets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--journal', type=Path, required=True)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--stage', type=Path, required=True)
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv[:1])
    widget = QtWidgets.QWidget(flags=QtCore.Qt.FramelessWindowHint)
    widget.setGeometry(1200, 0, 400, 900)
    widget.setStyleSheet('background:#172230;color:#edf3fa;font:12pt "DejaVu Sans";')
    layout = QtWidgets.QVBoxLayout(widget)
    heading = QtWidgets.QLabel('TURTLEBOT 4 · NAV2 + YOLO\nTô Văn Chúc · 23020727')
    heading.setStyleSheet('font-size:16pt;font-weight:bold;')
    heading.setWordWrap(True)
    layout.addWidget(heading)
    stage = QtWidgets.QLabel()
    stage.setWordWrap(True)
    layout.addWidget(stage)
    goals = QtWidgets.QLabel()
    goals.setWordWrap(True)
    layout.addWidget(goals)
    explanation = QtWidgets.QLabel(
        'Goal trong frame map (m, rad)\n'
        '1: (-2, -3, 0)\n2: (2, -3, 0)\n3: (-2, 4, π/2)\n\n'
        'Node waypoint_navigator gửi\n/navigate_to_pose theo thứ tự.\n\n'
        'Camera → cv_bridge → YOLOv8n\n→ /yolo/annotated_image\n\n'
        'Bảng này đọc JSON/log thật.\nStatus 4 = SUCCEEDED.')
    explanation.setStyleSheet('font-size:11pt;color:#b8cbe0;')
    layout.addWidget(explanation)
    layout.addWidget(QtWidgets.QLabel('LOG ĐANG CHẠY'))
    log_box = QtWidgets.QPlainTextEdit()
    log_box.setReadOnly(True)
    log_box.setStyleSheet('background:#0d1620;font:10pt "DejaVu Sans Mono";')
    layout.addWidget(log_box, 1)

    def update():
        stage.setText(args.stage.read_text() if args.stage.exists() else 'Đang khởi động')
        states = ['CHƯA GỬI'] * 3
        try:
            journal = json.loads(args.journal.read_text())
            for goal in journal['goals']:
                status = goal['status']
                states[goal['index'] - 1] = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, str(status))
            if journal.get('error'):
                stage.setText('Node báo lỗi: ' + journal['error'])
        except (OSError, ValueError, KeyError):
            pass
        goals.setText('\n'.join(f'Goal {i}: {value}' for i, value in enumerate(states, 1)))
        if args.log.exists():
            lines = args.log.read_text(errors='replace').splitlines()
            visible = []
            for line in lines:
                if '[waypoint_navigator]:' in line or '[yolo_detector]:' in line:
                    visible.append(re.sub(r'^.*?\[(waypoint_navigator|yolo_detector)\]: ', r'\1: ', line))
            text = '\n'.join(visible[-14:])
            if text != log_box.toPlainText():
                log_box.setPlainText(text)
                log_box.verticalScrollBar().setValue(log_box.verticalScrollBar().maximum())

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(500)
    update()
    widget.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
