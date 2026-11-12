#!/bin/bash
# Builds path3d and compares the global planners (A* with each heuristic, Dijkstra, Lazy Theta*, RRT,
# RRT-Connect, RRT*) on the benchmark scenes. No simulation is needed: run it in the container.
# It prints a table per scene and writes every row to a CSV file.
#
# Usage: run_benchmark.sh [-n|--no-build] [csv] [resolution]
#   csv         where to write the results. Default benchmark.csv in the repository root.
#   resolution  voxel size of the scene maps in meters. Default 0.5.

set -e

ROOT=$(realpath "$(dirname "$(readlink -f "$0")")/..")
cd "$ROOT"

BUILD=1
ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--no-build) BUILD=0; shift ;;
        *) ARGS+=("$1"); shift ;;
    esac
done
CSV="${ARGS[0]:-benchmark.csv}"
RESOLUTION="${ARGS[1]:-0.5}"

source /opt/ros/jazzy/setup.bash

if [[ $BUILD -eq 1 || ! -f install/setup.bash ]]; then
    colcon build --base-paths 3dpath --packages-select path3d
fi
source install/setup.bash

exec ros2 run path3d planner_benchmark "$CSV" "$RESOLUTION"
