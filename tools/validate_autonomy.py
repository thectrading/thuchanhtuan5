#!/usr/bin/env python3
"""Check recorded actions, concurrent camera detections and sampled hazards.

This validates real captured ROS data, and does not run or move the robot.
"""
import argparse
from collections import Counter
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation', type=Path, required=True)
    parser.add_argument('--telemetry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    navigation = json.loads(args.navigation.read_text())
    telemetry = json.loads(args.telemetry.read_text())
    goals = navigation['goals']
    assert navigation['all_succeeded'] and len(goals) >= 3
    assert all(g['status'] == 4 for g in goals)
    assert telemetry['rgb_frames'] > 0 and telemetry['annotated_frames'] > 0
    intervals = [(g['started_ros_s'], g['finished_ros_s']) for g in goals]

    def during_goals(stamp):
        return any(start <= stamp <= end for start, end in intervals)

    detections = [d for d in telemetry['detections'] if during_goals(
        d['header']['stamp']['sec'] + d['header']['stamp']['nanosec'] / 1e9)]
    assert detections, 'No positive YOLO detections while navigation was executing'
    hazards = Counter(kind for h in telemetry['hazards'] if during_goals(h['sim_time'])
                      for kind in h['types'])
    # Humble irobot_create_msgs/HazardDetection: BUMP=1, CLIFF=2, STALL=3, WHEEL_DROP=4.
    assert all(hazards[k] == 0 for k in (1, 2, 3, 4)), hazards
    classes = Counter(o['class_name'] for d in detections for o in d['objects'])
    summary = {'all_goals_succeeded': True, 'goal_wall_seconds': [g['elapsed_wall_s'] for g in goals],
               'rgb_frames': telemetry['rgb_frames'], 'annotated_frames': telemetry['annotated_frames'],
               'positive_detection_frames_during_navigation': len(detections),
               'predicted_class_counts': dict(classes),
               'max_confidence_during_navigation': max(o['confidence'] for d in detections for o in d['objects']),
               'hazards_during_navigation': dict(hazards),
               'bump_cliff_stall_wheel_drop_count': sum(hazards[k] for k in (1, 2, 3, 4)),
               'method': 'Read-only ROS records; predicted labels are not ground-truth class annotations. '
                         'Sampled hazard monitoring is not a continuous geometric collision proof.'}
    args.output.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
