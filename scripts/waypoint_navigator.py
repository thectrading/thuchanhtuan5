#!/usr/bin/env python3
"""Visit map poses sequentially using the Humble NavigateToPose action.

Requires an undocked robot, a verified initial pose, AMCL and Nav2. Never
publishes cmd_vel or overwrites localization. Wall time bounds remain effective
when Gazebo pauses. Only a SUCCEEDED result permits the next goal.
"""
import json
import math
from pathlib import Path
import signal
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from tf2_ros import Buffer, TransformException, TransformListener


def validated_goals(xs, ys, yaws):
    if not (len(xs) == len(ys) == len(yaws) and len(xs) >= 3):
        raise ValueError('goal_x, goal_y, goal_yaw must have equal lengths >= 3')
    goals = list(zip(xs, ys, yaws))
    if not all(math.isfinite(v) for goal in goals for v in goal):
        raise ValueError('Goal coordinates must be finite')
    return goals


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        defaults = {'goal_x': [-2.0, 2.0, -2.0], 'goal_y': [-3.0, -3.0, 4.0],
                    'goal_yaw': [0.0, 0.0, math.pi / 2], 'readiness_timeout': 120.0,
                    'goal_timeout': 300.0, 'pause_at_goal': 3.0,
                    'result_file': '/tmp/tb4_waypoints.json'}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.goals = validated_goals(*(self.value(k) for k in ('goal_x', 'goal_y', 'goal_yaw')))
        for name in ('readiness_timeout', 'goal_timeout', 'pause_at_goal'):
            v = self.value(name)
            if not math.isfinite(v) or v < 0 or (name != 'pause_at_goal' and v == 0):
                raise ValueError(f'Invalid {name}')
        self.output = Path(self.value('result_file')).expanduser()
        self.evidence = {'method': 'waypoint_navigator /navigate_to_pose',
                         'frame': 'map', 'all_succeeded': False, 'goals': []}
        self.pose = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.on_pose,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.lifecycle = {name: self.create_client(GetState, name + '/get_state')
                          for name in ('amcl', 'bt_navigator', 'planner_server', 'controller_server')}
        self.goal_handle = None
        self.pending_goal = None
        self.result_future = None
        self.last_feedback = 0.0

    def value(self, name):
        return self.get_parameter(name).value

    def on_pose(self, msg):
        self.pose = msg

    def wait(self, predicate, timeout, description):
        end = time.monotonic() + timeout
        while rclpy.ok() and not predicate():
            if time.monotonic() >= end:
                raise TimeoutError(description)
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            raise RuntimeError('ROS context stopped')

    def future_result(self, future, timeout, description):
        self.wait(future.done, timeout, description)
        return future.result()

    def localized(self):
        if self.pose is None or self.pose.header.frame_id != 'map':
            return False
        now = self.get_clock().now().nanoseconds / 1e9
        if now <= 0:
            return False
        try:
            tf = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
            return -2.0 <= now - stamp <= 5.0
        except TransformException:
            return False

    def ready(self):
        self.get_logger().info('Waiting for active AMCL/Nav2 and map -> base_link. Verify scan alignment in RViz.')
        deadline = time.monotonic() + self.value('readiness_timeout')
        def remaining():
            value = deadline - time.monotonic()
            if value <= 0:
                raise TimeoutError('AMCL/Nav2 readiness timed out; check initial pose, TF and /clock')
            return value
        for name, client in self.lifecycle.items():
            while True:
                self.wait(client.service_is_ready, remaining(), f'{name} service unavailable')
                state = self.future_result(client.call_async(GetState.Request()), remaining(), name)
                if state.current_state.id == 3:
                    break
                rclpy.spin_once(self, timeout_sec=min(0.5, remaining()))
        self.wait(lambda: self.client.server_is_ready() and self.localized(), remaining(),
                  'Missing action server or localization; set 2D Pose Estimate and check TF')

    def feedback(self, msg):
        if time.monotonic() - self.last_feedback > 2.0:
            f = msg.feedback
            self.get_logger().info(f'Distance remaining: {f.distance_remaining:.2f} m; recoveries: {f.number_of_recoveries}')
            self.last_feedback = time.monotonic()

    def save(self):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temp = self.output.with_suffix(self.output.suffix + '.tmp')
        temp.write_text(json.dumps(self.evidence, indent=2) + '\n')
        temp.replace(self.output)

    def run(self):
        self.save()
        self.ready()
        for index, (x, y, yaw) in enumerate(self.goals, 1):
            self.wait(self.localized, 10.0, 'Localization TF became unavailable')
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation.z = math.sin(yaw / 2)
            goal.pose.pose.orientation.w = math.cos(yaw / 2)
            entry = {'index': index, 'target': [x, y, yaw], 'status': 'SENDING',
                     'started_ros_s': self.get_clock().now().nanoseconds / 1e9}
            self.evidence['goals'].append(entry)
            self.save()
            self.get_logger().info(f'Goal {index}/{len(self.goals)}: ({x}, {y}, {yaw}) in map')
            started = time.monotonic()
            self.pending_goal = self.client.send_goal_async(goal, feedback_callback=self.feedback)
            self.goal_handle = self.future_result(self.pending_goal, 15.0, 'Goal acknowledgement timeout')
            self.pending_goal = None
            if not self.goal_handle.accepted:
                self.goal_handle = None
                entry['status'] = 'REJECTED'
                raise RuntimeError(f'Goal {index} rejected')
            entry['status'] = 'ACCEPTED'
            self.save()
            self.result_future = self.goal_handle.get_result_async()
            result = self.future_result(self.result_future, self.value('goal_timeout'),
                                        f'Goal {index} timed out')
            self.goal_handle = None
            self.result_future = None
            entry.update(status=result.status, elapsed_wall_s=round(time.monotonic() - started, 3),
                         finished_ros_s=self.get_clock().now().nanoseconds / 1e9)
            self.save()
            if result.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'Goal {index} failed: action status {result.status}')
            self.get_logger().info(f'Goal {index} SUCCEEDED')
            until = time.monotonic() + self.value('pause_at_goal')
            self.wait(lambda: time.monotonic() >= until, self.value('pause_at_goal') + 1, 'Pause')
        self.evidence['all_succeeded'] = True
        self.save()
        self.get_logger().info(f'All {len(self.goals)} goals SUCCEEDED. Results: {self.output}')

    def cancel(self):
        # Also handle Ctrl+C while the server is acknowledging the goal.
        try:
            if self.pending_goal is not None:
                self.goal_handle = self.future_result(self.pending_goal, 5.0, 'Goal acknowledgement during shutdown')
            if self.goal_handle is not None and self.goal_handle.accepted:
                response = self.future_result(self.goal_handle.cancel_goal_async(), 5.0, 'Cancel acknowledgement timeout')
                if response.goals_canceling:
                    future = self.result_future or self.goal_handle.get_result_async()
                    result = self.future_result(future, 5.0, 'Canceled goal result timeout')
                    self.evidence['goals'][-1]['status'] = result.status
                else:
                    self.get_logger().warning('Cancel not accepted; inspect /navigate_to_pose status')
        except Exception as exc:
            self.get_logger().error(f'Could not confirm navigation stopped: {exc}')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    node = None
    code = 1
    try:
        node = WaypointNavigator()
        node.run()
        code = 0
    except (Exception, KeyboardInterrupt) as exc:
        if node:
            node.get_logger().error(str(exc) or 'Interrupted; canceling navigation')
            node.evidence['error'] = str(exc) or 'Interrupted'
            node.cancel()
            node.save()
        else:
            print(f'Navigator initialization failed: {exc}', file=sys.stderr)
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
