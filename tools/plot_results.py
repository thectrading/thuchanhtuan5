#!/usr/bin/env python3
"""Plot the saved SLAM map and, if present, real AMCL navigation telemetry."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


def main():
    root=Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--navigation', type=Path, default=root/'artifacts/navigation_results.json')
    parser.add_argument('--telemetry', type=Path)
    parser.add_argument('--output', type=Path, default=root/'artifacts/map_preview.png')
    args = parser.parse_args()
    metadata=yaml.safe_load((root/'maps/map.yaml').read_text())
    pixels=np.array(Image.open(root/'maps'/metadata['image']))
    x,y,_=metadata['origin'];resolution=metadata['resolution']
    fig,ax=plt.subplots(figsize=(7,7))
    ax.imshow(pixels,cmap='gray',vmin=0,vmax=254,
              extent=(x,x+pixels.shape[1]*resolution,y,y+pixels.shape[0]*resolution),origin='upper')
    result=args.navigation
    if result.exists():
        data=json.loads(result.read_text())
        telemetry = json.loads(args.telemetry.read_text()) if args.telemetry else data
        path=np.array(telemetry.get('amcl_trajectory',[]))
        if path.size:ax.plot(path[:,1],path[:,2],color='#e27716',lw=1.5,label='AMCL trajectory')
        for i,goal in enumerate(data['goals'],1):
            gx,gy,_=goal['target']
            ax.plot(gx,gy,'o',color='#26834b' if goal.get('status')==4 else '#a32d34')
            ax.annotate(f'G{i}: '+('SUCCEEDED' if goal.get('status')==4 else str(goal.get('status','pending'))),
                        (gx,gy),xytext=(5,8),textcoords='offset points',fontsize=8)
        if path.size:ax.legend(loc='lower left')
    ax.set(xlabel='map x (m)',ylabel='map y (m)',title='SLAM Toolbox map / Nav2 integration evidence')
    ax.set_aspect('equal');ax.grid(alpha=.15)
    fig.text(.5,.018,'White: free | Black: occupied | Grey: unknown. Plot from ROS data; not an RViz screenshot.',
             ha='center',fontsize=7)
    fig.tight_layout(rect=(0,.035,1,1))
    out=args.output;fig.savefig(out,dpi=160)
    print(out)


if __name__=='__main__':main()
