#!/usr/bin/env python3
"""Validate recorded Nav2 results against read-only simulation telemetry."""
import collections
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from irobot_create_msgs.msg import HazardDetection


def main():
    root=Path(__file__).resolve().parents[1]
    data=json.loads((root/'artifacts/navigation_results.json').read_text())
    assert data.get('all_succeeded') and len(data['goals'])==3
    assert all(g['status']==4 for g in data['goals'])
    counts=collections.Counter(t for h in data['hazards'] for t in h['types'])
    for kind in (HazardDetection.BUMP,HazardDetection.CLIFF,HazardDetection.STALL,HazardDetection.WHEEL_DROP):
        assert counts[kind]==0,counts
    boxes=[]
    world=ET.parse(Path(get_package_share_directory('turtlebot4_ignition_bringup'))/'worlds/maze.sdf')
    for model in world.findall('.//world/model'):
        if model.get('name') not in ('obstacles','border'):continue
        for link in model.findall('link'):
            x,y,*_=map(float,link.findtext('pose').split())
            sx,sy,*_=map(float,link.findtext('collision/geometry/box/size').split())
            boxes.append((x-sx/2,x+sx/2,y-sy/2,y+sy/2))
    clearance=[]
    for stamp,x,y in data['gazebo_trajectory']:
        clearance.append(min(math.hypot(max(xmin-x,0,x-xmax),max(ymin-y,0,y-ymax))
                             for xmin,xmax,ymin,ymax in boxes))
    assert clearance and min(clearance)>.175,clearance
    summary={'all_three_succeeded':True,'hazard_detection_counts':dict(counts),
        'backup_limit_count':counts[HazardDetection.BACKUP_LIMIT],
        'bump_count':counts[HazardDetection.BUMP],'cliff_count':counts[HazardDetection.CLIFF],
        'stall_count':counts[HazardDetection.STALL],'wheel_drop_count':counts[HazardDetection.WHEEL_DROP],
        'ground_truth_samples':len(clearance),'min_sampled_center_to_maze_wall_m':min(clearance),
        'initial_localization':data['initial_localization'],
        'goal_map_xy_errors_m':[g['xy_error_m'] for g in data['goals']],
        'amcl_gazebo_xy_differences_m':[g['amcl_gazebo_xy_difference_m'] for g in data['goals']],
        'method':'Read-only sampled verification against maze boxes; not a continuous collision proof and not RViz Goal video.'}
    for goal in data['goals']:
        assert goal['xy_error_m']<=.25 and goal['yaw_error_rad']<=.25
        assert goal['amcl_gazebo_xy_difference_m']<.5
    (root/'artifacts/navigation_validation.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
