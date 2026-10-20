#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: ros2 run turtlebot4_slam_nav save_map.sh /absolute/path/maps/map" >&2
  exit 2
fi
task_map_prefix="$1"
if [[ "$task_map_prefix" != /* ]]; then
  echo 'Use an absolute output path without .yaml or .pgm.' >&2
  exit 2
fi
mkdir -p "$(dirname "$task_map_prefix")"
if [[ -e "${task_map_prefix}.yaml" || -e "${task_map_prefix}.pgm" ]]; then
  echo 'Output map already exists. Use a new prefix, or archive the previous map first.' >&2
  exit 2
fi
# Humble writes unknown PGM pixels as 205 (occupancy ~= 0.196078).
# A free threshold of 0.196 preserves them as unknown when loading the PGM.
ros2 run nav2_map_server map_saver_cli -f "$task_map_prefix" --fmt pgm --free 0.196 --occ 0.65 \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
test -s "${task_map_prefix}.yaml"
test -s "${task_map_prefix}.pgm"
echo "Saved ${task_map_prefix}.yaml and ${task_map_prefix}.pgm"
