#!/usr/bin/env python3
"""Record this host's RViz X11 window and place three goals with its Nav2 Goal tool.

Requires GStreamer ximagesrc/vp8enc, XTest, xdotool, rclpy and the visible 1100x800 demo.rviz
layout at scale 24, center (0,0), angle 0. Captures real RViz pixels; no simulated
video or direct NavigateToPose goal requests. Keep the mouse idle during clicks.
"""
import argparse
import collections
import ctypes
import ctypes.util
import json
import math
import re
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import HazardDetectionVector
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener


class Mouse:
    def __init__(self, window, xdotool='xdotool'):
        self.window = window
        self.xdotool = xdotool
        self.x = ctypes.CDLL(ctypes.util.find_library('X11'))
        self.test = ctypes.CDLL(ctypes.util.find_library('Xtst'))
        self.x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x.XOpenDisplay.restype = ctypes.c_void_p
        self.display = self.x.XOpenDisplay(None)
        if not self.display: raise RuntimeError('No X11 display')
        self.x.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x.XSetInputFocus.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int, ctypes.c_ulong]
        self.x.XFlush.argtypes = [ctypes.c_void_p]
        self.x.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.test.XTestFakeMotionEvent.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_ulong]
        self.test.XTestFakeButtonEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_int, ctypes.c_ulong]

    def geometry(self):
        text = subprocess.check_output(['xwininfo', '-id', hex(self.window)], text=True)
        if '- RViz' not in text or 'IsViewable' not in text:
            raise RuntimeError('RViz target is not visible')
        def read(key): return int(re.search(re.escape(key) + r':\s*(-?\d+)', text)[1])
        if (read('Width'), read('Height')) != (1100, 800):
            raise RuntimeError('RViz resized; stopping rather than clicking an uncertain location')
        return read('Absolute upper-left X'), read('Absolute upper-left Y')

    def move(self, x, y):
        active=subprocess.check_output([self.xdotool,'getactivewindow'],text=True,stderr=subprocess.DEVNULL).strip().splitlines()[-1]
        if int(active)!=self.window:raise RuntimeError('RViz lost focus; stopping UI input')
        ox, oy = self.geometry()
        self.test.XTestFakeMotionEvent(self.display, -1, round(ox+x), round(oy+y), 0)
        self.x.XFlush(self.display); time.sleep(.15)

    def button(self, down):
        self.test.XTestFakeButtonEvent(self.display, 1, int(down), 0)
        self.x.XFlush(self.display); time.sleep(.15)

    def goal(self, x, y, yaw):
        self.geometry()
        # On Wayland, XRaiseWindow/XSetInputFocus alone do not activate the
        # compositor's surface. EWMH activation must happen before global input.
        subprocess.run([self.xdotool,'windowmap',hex(self.window),'windowraise',hex(self.window),
                        'windowfocus',hex(self.window),'windowactivate',hex(self.window)],check=True,timeout=5,
                       stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            active=subprocess.check_output([self.xdotool,'getactivewindow'],text=True,stderr=subprocess.DEVNULL).strip().splitlines()[-1]
            if int(active)==self.window:break
            time.sleep(.1)
        else:raise RuntimeError('Cannot activate RViz for safe UI input')
        time.sleep(.4)
        self.move(720, 40); self.button(True); self.button(False)  # Nav2 Goal toolbar
        px = 490 + 585/2 + x*585/24
        py = 70 + 692/2 - y*585/24
        self.move(px, py); self.button(True)
        for fraction in (.25, .5, .75, 1):
            self.move(px+40*math.cos(yaw)*fraction, py-40*math.sin(yaw)*fraction)
        self.button(False)

    def close(self): self.x.XCloseDisplay(self.display)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--window', type=lambda x:int(x,0), required=True)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--xdotool', default='xdotool')
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--third-goal-y', type=float, default=4.0)
    args = parser.parse_args()
    if args.video.exists() or args.evidence.exists(): raise FileExistsError('Use a new output filename')
    args.video.parent.mkdir(parents=True, exist_ok=True)
    args.evidence.parent.mkdir(parents=True, exist_ok=True)
    mouse = Mouse(args.window,args.xdotool); mouse.geometry()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('rviz_video_monitor', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(); listener = TransformListener(buffer, node)
    statuses = {}; latest = {}; active_id = None; recorder = None
    evidence = {'method':'Real RViz window capture; XTest clicks on Nav2 Goal; monitor only, no direct navigation goal requests',
                'video':str(args.video), 'fps':args.fps, 'playback_speed':1, 'goals':[], 'hazards':[], 'amcl_trajectory':[], 'gazebo_trajectory':[]}
    recording = [False]
    def on_status(message):
        for status in message.status_list: statuses[bytes(status.goal_info.goal_id.uuid).hex()] = status.status
    def on_amcl(message):
        pose = message.pose.pose; latest['amcl'] = pose
        if recording[0]:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
            evidence['amcl_trajectory'].append([stamp,pose.position.x,pose.position.y])
    gt_stamp = [0.0]
    def on_gt(message):
        latest['ground_truth'] = message.pose.pose
        stamp = message.header.stamp.sec + message.header.stamp.nanosec/1e9
        if recording[0] and stamp-gt_stamp[0]>.5:
            evidence['gazebo_trajectory'].append([stamp,message.pose.pose.position.x,message.pose.pose.position.y]);gt_stamp[0]=stamp
    def on_hazard(message):
        if recording[0] and message.detections:
            evidence['hazards'].append({'sim_time':message.header.stamp.sec+message.header.stamp.nanosec/1e9,
                                      'types':[item.type for item in message.detections]})
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',on_status,qos),
                     node.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',on_amcl,qos_profile_sensor_data),
                     node.create_subscription(Odometry,'/sim_ground_truth_pose',on_gt,qos_profile_sensor_data),
                     node.create_subscription(HazardDetectionVector,'/hazard_detection',on_hazard,qos_profile_sensor_data)]
    def spin(seconds):
        until = time.monotonic()+seconds
        while time.monotonic()<until: rclpy.spin_once(node, timeout_sec=.1)
    try:
        spin(4)
        if node.count_publishers('/clock') != 1: raise RuntimeError('Expected exactly one simulation clock')
        if not buffer.can_transform('map','base_link',rclpy.time.Time()): raise RuntimeError('No map to robot TF')
        if 'ground_truth' in latest:
            transform=buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform
            pose=latest['ground_truth'];q=pose.orientation
            actual_yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            q=transform.rotation;estimated_yaw=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
            xy_error=math.hypot(pose.position.x-transform.translation.x,pose.position.y-transform.translation.y)
            yaw_error=abs(math.atan2(math.sin(actual_yaw-estimated_yaw),math.cos(actual_yaw-estimated_yaw)))
            evidence['initial_localization']={'xy_difference_m':xy_error,'yaw_difference_rad':yaw_error}
            if xy_error>.5 or yaw_error>.5:raise RuntimeError('Initial localization disagrees with Gazebo')
        for name in ['amcl','planner_server','controller_server','bt_navigator']:
            client = node.create_client(GetState,f'/{name}/get_state')
            if not client.wait_for_service(timeout_sec=5): raise RuntimeError(f'No {name} lifecycle service')
            future = client.call_async(GetState.Request())
            until=time.monotonic()+5
            while not future.done() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
            if not future.done() or future.result().current_state.id!=3: raise RuntimeError(f'{name} is not active')
        # A simulator clock reset can clear RViz's map display without resending
        # the latched map. Check the pixels before attempting a whole recording.
        import cv2
        preview=args.evidence.with_suffix('.preflight.png')
        subprocess.run(['gst-launch-1.0','-q','ximagesrc',f'xid={args.window}',
                        'use-damage=false','show-pointer=false','num-buffers=1','!',
                        'videoconvert','!','pngenc','!','filesink',f'location={preview}'],check=True,timeout=10)
        image=cv2.imread(str(preview))
        if image is None or (image[70:762,490:1075].max(axis=2)>180).mean()<.10:
            raise RuntimeError('Map is not visible in RViz; toggle the Map display or reopen RViz first')
        recorder = subprocess.Popen(['gst-launch-1.0','-q','-e','ximagesrc',f'xid={args.window}',
                'use-damage=false','show-pointer=true','!',f'video/x-raw,framerate={args.fps}/1','!','videoconvert','!',
                'vp8enc','deadline=1','cpu-used=8','threads=1','target-bitrate=1200000','!',
                'webmmux','!','filesink',f'location={args.video}'])
        recording[0]=True; start_video=time.monotonic();spin(5)
        for index, (x,y,yaw) in enumerate([(-2.,-3.,0.),(2.,-3.,0.),(-2.,args.third_goal_y,math.pi/2)],1):
            if recorder.poll() is not None: raise RuntimeError('Screen recording stopped')
            before = set(statuses); start=time.monotonic()
            mouse.goal(x,y,yaw)
            print(f'RVIZ GOAL {index} CLICKED: approximate map pose ({x}, {y}, {yaw:.3f})',flush=True)
            until=time.monotonic()+15
            while not set(statuses)-before and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
            new=set(statuses)-before
            if len(new)!=1: raise RuntimeError(f'Expected one new RViz navigation goal, got {len(new)}')
            active_id=new.pop()
            entry={'nominal_clicked_pose':[x,y,yaw],'goal_id':active_id};evidence['goals'].append(entry)
            last_log=0.0
            while statuses.get(active_id) not in [4,5,6]:
                if time.monotonic()-start>500: raise TimeoutError('Goal timed out')
                if recorder.poll() is not None: raise RuntimeError('Recorder failed during navigation')
                rclpy.spin_once(node,timeout_sec=.1)
                if time.monotonic()-last_log>15:
                    if 'amcl' in latest:
                        pos=latest['amcl'].position;print(f'RVIZ GOAL {index}: robot ({pos.x:.2f}, {pos.y:.2f})',flush=True)
                    last_log=time.monotonic()
            entry['status']=statuses[active_id];entry['elapsed_wall_s']=round(time.monotonic()-start,3)
            transform=buffer.lookup_transform('map','base_link',rclpy.time.Time()).transform
            q=transform.rotation;angle=math.atan2(2*q.w*q.z,1-2*q.z*q.z)
            entry['final_map_pose']=[transform.translation.x,transform.translation.y,angle]
            if 'ground_truth' in latest:
                actual=latest['ground_truth'].position
                entry['amcl_gazebo_difference_m']=math.hypot(actual.x-transform.translation.x,actual.y-transform.translation.y)
            print(f'RVIZ GOAL {index} RESULT: {entry["status"]} (4=SUCCEEDED)',flush=True)
            active_id=None
            if entry['status']!=4:raise RuntimeError('RViz goal did not succeed')
            if entry.get('amcl_gazebo_difference_m',0)>.5:raise RuntimeError('Final localization disagrees with Gazebo')
            spin(5)
        evidence['all_three_succeeded']=True
        evidence['duration_wall_s']=round(time.monotonic()-start_video,3)
        evidence['hazard_counts']=dict(collections.Counter(kind for event in evidence['hazards'] for kind in event['types']))
    except BaseException as exc:
        evidence['error']=str(exc)
        if active_id is not None:
            client=node.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
            if client.wait_for_service(timeout_sec=3):
                request=CancelGoal.Request();request.goal_info.goal_id.uuid=list(bytes.fromhex(active_id))
                future=client.call_async(request);spin(3)
        raise
    finally:
        recording[0]=False
        if recorder is not None and recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:recorder.wait(timeout=20)
            except subprocess.TimeoutExpired:recorder.terminate();recorder.wait(timeout=5)
        args.evidence.write_text(json.dumps(evidence,indent=2))
        mouse.close();node.destroy_node();rclpy.shutdown()
        print('Video:',args.video,flush=True)


if __name__=='__main__':main()
