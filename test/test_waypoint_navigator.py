"""Exercise the installed action client against a real ROS action server.

No Gazebo claims: these tests verify sequencing, failure and cancellation only.
Run with a separate ROS_DOMAIN_ID from the simulator.
"""
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import threading
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_ros import TransformBroadcaster

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/waypoint_navigator.py'


def test_invalid_goals():
    spec = importlib.util.spec_from_file_location('waypoints', SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for coords in [([0.0], [0.0], [0.0]), ([0.0] * 3, [0.0] * 2, [0.0] * 3),
                   ([math.nan] * 3, [0.0] * 3, [0.0] * 3)]:
        with pytest.raises(ValueError):
            module.validated_goals(*coords)


@pytest.mark.parametrize('mode,expected_count,success', [
    ('success', 3, True), ('abort', 1, False), ('timeout', 1, False), ('no_pose', 0, False)])
def test_real_action_sequence(tmp_path, mode, expected_count, success):
    rclpy.init()
    node = Node('test_nav2_server')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    seen = []
    canceled = []
    broadcaster = TransformBroadcaster(node)
    pub = node.create_publisher(PoseWithCovarianceStamped, 'amcl_pose',
                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def sensors():
        if mode == 'no_pose':
            return
        stamp = node.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id, tf.header.stamp = 'map', 'base_link', stamp
        tf.transform.rotation.w = 1.0
        broadcaster.sendTransform(tf)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id, pose.header.stamp = 'map', stamp
        pose.pose.pose.orientation.w = 1.0
        pub.publish(pose)

    def state(_, response):
        response.current_state.id = 3
        return response

    services = [node.create_service(GetState, name + '/get_state', state)
                for name in ('amcl', 'bt_navigator', 'planner_server', 'controller_server')]

    def execute(handle):
        pose = handle.request.pose
        seen.append([pose.pose.position.x, pose.pose.position.y])
        assert pose.header.frame_id == 'map'
        assert abs(pose.pose.orientation.z ** 2 + pose.pose.orientation.w ** 2 - 1) < 1e-6
        if mode == 'timeout':
            end = time.monotonic() + 10
            while time.monotonic() < end and not handle.is_cancel_requested:
                time.sleep(0.02)
            if handle.is_cancel_requested:
                canceled.append(True)
                handle.canceled()
            else:
                handle.abort()
        elif mode == 'abort':
            handle.abort()
        else:
            handle.succeed()
        return NavigateToPose.Result()

    # Reentrant group permits cancel processing during the blocking test callback.
    from rclpy.callback_groups import ReentrantCallbackGroup
    server = ActionServer(node, NavigateToPose, 'navigate_to_pose', execute,
                          cancel_callback=lambda _: CancelResponse.ACCEPT,
                          callback_group=ReentrantCallbackGroup())
    timer = node.create_timer(0.05, sensors)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    output = tmp_path / 'result.json'
    try:
        result = subprocess.run([
            '/usr/bin/python3', str(SCRIPT), '--ros-args', '-p', 'use_sim_time:=false',
            '-p', 'readiness_timeout:=4.0', '-p', 'goal_timeout:=0.5', '-p', 'pause_at_goal:=0.0',
            '-p', f'result_file:={output}'], capture_output=True, text=True, timeout=20)
        assert output.exists(), result.stdout + result.stderr
        data = json.loads(output.read_text())
        assert (result.returncode == 0) == success, result.stdout + result.stderr
        assert data['all_succeeded'] == success
        assert len(seen) == expected_count
        if success:
            assert seen == [[-2.0, -3.0], [2.0, -3.0], [-2.0, 4.0]]
            assert [g['status'] for g in data['goals']] == [4, 4, 4]
        if mode == 'timeout':
            assert canceled and data['goals'][0]['status'] == 5
    finally:
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        server.destroy()
        node.destroy_node()
        rclpy.shutdown()
