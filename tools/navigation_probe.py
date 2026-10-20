#!/usr/bin/env python3
"""Check three real Nav2 actions on the saved maze map, without publishing cmd_vel.

This is integration evidence, NOT a replacement for the RViz Nav2 Goal video.
Run after restarting the world, loading the saved map and successfully undocking.
"""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from irobot_create_msgs.msg import HazardDetectionVector
from tf2_ros import Buffer, TransformListener


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--initial-x',type=float,default=-.3)
    parser.add_argument('--initial-y',type=float,default=0.0)
    parser.add_argument('--initial-yaw',type=float,default=math.pi,
                        help='Default: heading after the official undock action turns the robot around')
    parser.add_argument('--timeout',type=float,default=500)
    args=parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=rclpy.create_node('navigation_action_probe',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    tf=Buffer();listener=TransformListener(tf,node)
    evidence={'method':'Nav2 action integration test; goals were not clicked in RViz', 'goals':[], 'hazards':[], 'odom':[], 'amcl_trajectory':[], 'gazebo_trajectory':[]}
    ground_truth=[]
    running=[False]
    def hazards(message):
        if running[0] and message.detections:
            evidence['hazards'].append({'sim_time':message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                                       'types':[d.type for d in message.detections]})
    last=[0.0]
    def odom(message):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if running[0] and stamp-last[0]>.5:
            p=message.pose.pose.position;evidence['odom'].append([stamp,p.x,p.y]);last[0]=stamp
    def amcl(message):
        if running[0]:
            p=message.pose.pose.position
            evidence['amcl_trajectory'].append([message.header.stamp.sec+message.header.stamp.nanosec/1e9,p.x,p.y])
    last_gt=[0.0]
    def record_gt(message):
        ground_truth[:] = [message]
        stamp=message.header.stamp.sec+message.header.stamp.nanosec/1e9
        if running[0] and stamp-last_gt[0]>.5:
            p=message.pose.pose.position;evidence['gazebo_trajectory'].append([stamp,p.x,p.y]);last_gt[0]=stamp
    subs=[node.create_subscription(HazardDetectionVector,'hazard_detection',hazards,qos_profile_sensor_data),
          node.create_subscription(Odometry,'odom',odom,qos_profile_sensor_data),
          node.create_subscription(PoseWithCovarianceStamped,'amcl_pose',amcl,qos_profile_sensor_data),
          node.create_subscription(Odometry,'sim_ground_truth_pose',record_gt,qos_profile_sensor_data)]
    publisher=node.create_publisher(PoseWithCovarianceStamped,'initialpose',10)
    client=ActionClient(node,NavigateToPose,'navigate_to_pose')
    def spin_until(predicate,seconds):
        deadline=time.monotonic()+seconds
        while not predicate() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not predicate():raise TimeoutError('ROS response/TF timeout')
    def wait_future(future,seconds):
        spin_until(future.done,seconds)
        return future.result()
    active_handle=None
    try:
        # Publishing the same estimate a few times handles discovery/AMCL startup.
        for _ in range(5):
            p=PoseWithCovarianceStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg()
            p.pose.pose.position.x=args.initial_x;p.pose.pose.position.y=args.initial_y
            p.pose.pose.orientation.z=math.sin(args.initial_yaw/2);p.pose.pose.orientation.w=math.cos(args.initial_yaw/2)
            p.pose.covariance[0]=.04;p.pose.covariance[7]=.04;p.pose.covariance[35]=.04
            publisher.publish(p)
            for _ in range(5):rclpy.spin_once(node,timeout_sec=.1)
        spin_until(lambda:tf.can_transform('map','base_link',rclpy.time.Time()),30)
        evidence['lifecycle']={}
        for name in ('amcl','planner_server','controller_server','bt_navigator'):
            state_client=node.create_client(GetState,f'/{name}/get_state')
            if not state_client.wait_for_service(timeout_sec=10):raise TimeoutError(f'No lifecycle service: {name}')
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                state=wait_future(state_client.call_async(GetState.Request()),5).current_state
                if state.id==3:
                    evidence['lifecycle'][name]=state.label
                    break
                for _ in range(5):rclpy.spin_once(node,timeout_sec=.1)
            else:raise TimeoutError(f'Lifecycle not active: {name}')
        if not client.wait_for_server(timeout_sec=40):raise TimeoutError('No NavigateToPose server')
        if ground_truth:
            measured=tf.lookup_transform('map','base_link',rclpy.time.Time()).transform
            actual=ground_truth[-1].pose.pose
            q=measured.rotation;estimated_yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
            q=actual.orientation;actual_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            xy_difference=math.hypot(actual.position.x-measured.translation.x,actual.position.y-measured.translation.y)
            yaw_difference=abs(math.atan2(math.sin(actual_yaw-estimated_yaw),math.cos(actual_yaw-estimated_yaw)))
            evidence['initial_localization']={'xy_difference_m':xy_difference,'yaw_difference_rad':yaw_difference}
            print('INITIAL_LOCALIZATION',json.dumps(evidence['initial_localization']),flush=True)
            if xy_difference>.5 or yaw_difference>.5:raise RuntimeError('Initial localization disagrees with Gazebo; inspect initial pose before navigation')
        for x,y,yaw in [(-2.,-3.,0.),(2.,-3.,0.),(-2.,4.,math.pi/2)]:
            goal=NavigateToPose.Goal();goal.pose=PoseStamped();goal.pose.header.frame_id='map'
            goal.pose.header.stamp=node.get_clock().now().to_msg()
            goal.pose.pose.position.x=x;goal.pose.pose.position.y=y
            goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
            entry={'target':[x,y,yaw]};evidence['goals'].append(entry);running[0]=True;start=time.monotonic()
            print(f'GOAL {len(evidence["goals"])} START: x={x}, y={y}, yaw={yaw:.3f}',flush=True)
            last_feedback=[0.0]
            def feedback(message):
                now=time.monotonic()
                if now-last_feedback[0]>=10:
                    print(f'GOAL {len(evidence["goals"])}: distance remaining {message.feedback.distance_remaining:.2f} m',flush=True)
                    last_feedback[0]=now
            active_handle=wait_future(client.send_goal_async(goal,feedback_callback=feedback),30)
            entry['accepted']=active_handle.accepted
            if not active_handle.accepted:raise RuntimeError('Goal rejected')
            try:result=wait_future(active_handle.get_result_async(),args.timeout)
            except TimeoutError:
                wait_future(active_handle.cancel_goal_async(),10)
                entry['status']='TIMEOUT';raise
            entry['status']=result.status;entry['elapsed_wall_s']=round(time.monotonic()-start,3)
            transform=tf.lookup_transform('map','base_link',rclpy.time.Time());pos=transform.transform.translation
            q=transform.transform.rotation
            measured_yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
            entry['final_map_pose']=[pos.x,pos.y,measured_yaw]
            entry['xy_error_m']=math.hypot(pos.x-x,pos.y-y)
            entry['yaw_error_rad']=abs(math.atan2(math.sin(measured_yaw-yaw),math.cos(measured_yaw-yaw)))
            if ground_truth:
                actual=ground_truth[-1].pose.pose.position
                entry['gazebo_position']=[actual.x,actual.y]
                entry['amcl_gazebo_xy_difference_m']=math.hypot(actual.x-pos.x,actual.y-pos.y)
                # Ground truth is read only, never an input to AMCL or the controller.
                ground_truth.clear()
            print(json.dumps(entry),flush=True);running[0]=False;active_handle=None
            if result.status!=4:raise RuntimeError(f'Navigation failed with status {result.status}')
            if entry.get('amcl_gazebo_xy_difference_m',0)>.5:raise RuntimeError('Goal status succeeded, but localization disagrees with Gazebo')
        evidence['all_succeeded']=True
    finally:
        if active_handle is not None:
            try:wait_future(active_handle.cancel_goal_async(),10)
            except Exception:pass
        Path(args.output).write_text(json.dumps(evidence,indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
