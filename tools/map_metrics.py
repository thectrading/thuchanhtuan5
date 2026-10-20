#!/usr/bin/env python3
"""Read-only coverage estimate against the built-in maze's free floor.

This does not create or alter maps. The estimate assumes map and world initially
align at the default spawn; it does not measure every wall error or prove safe navigation.
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
from ament_index_python.packages import get_package_share_directory


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map')
    args=parser.parse_args()
    path=Path(args.map);meta=yaml.safe_load(path.read_text());pixels=np.array(Image.open(path.parent/meta['image']))
    grid=.05
    x,y=np.meshgrid(np.arange(-10+grid/2,10,grid),np.arange(-10+grid/2,10,grid))
    free=np.ones(x.shape,dtype=bool)
    world=ET.parse(Path(get_package_share_directory('turtlebot4_ignition_bringup'))/'worlds/maze.sdf')
    for model in world.findall('.//world/model'):
        if model.get('name')!='obstacles':continue
        for link in model.findall('link'):
            pose=[float(v) for v in link.findtext('pose').split()]
            size=[float(v) for v in link.findtext('collision/geometry/box/size').split()]
            free &= ~((abs(x-pose[0])<=size[0]/2)&(abs(y-pose[1])<=size[1]/2))
    col=np.floor((x-meta['origin'][0])/meta['resolution']).astype(int)
    row=pixels.shape[0]-1-np.floor((y-meta['origin'][1])/meta['resolution']).astype(int)
    valid=(col>=0)&(col<pixels.shape[1])&(row>=0)&(row<pixels.shape[0])
    observed=np.full(x.shape,205,dtype=np.uint8)
    observed[valid]=pixels[row[valid],col[valid]]
    print(json.dumps({'map':str(path),'pixels':[pixels.shape[1],pixels.shape[0]],
        'resolution_m':meta['resolution'],'origin':meta['origin'],
        'maze_free_floor_m2':round(float(np.count_nonzero(free)*grid*grid),2),
        'observed_free_floor_percent':round(float(np.count_nonzero((observed!=205)&free)/np.count_nonzero(free)*100),2),
        'assumption':'Default maze, map/world alignment at origin; read-only approximate coverage, not acceptance of wall accuracy.'},indent=2))


if __name__=='__main__':main()
