#!/bin/bash
# Builds path3d and runs the line planner demo: the drone flies a straight line from where it is.
# The drone must already be flying in GUIDED mode: start the simulation (./scripts/start.sh), take
# off (QGroundControl's Takeoff, or in the `ardupilot` window: `mode guided`, `arm throttle`,
# `takeoff 5`), then run this in the `planner` tmux window.
#
# Usage: run_line_planner.sh [-n|--no-build] [yaw] [distance] [world]
#   yaw       direction of the line in degrees, counter-clockwise from east (0 = east, 90 = north).
#             Default 0.
#   distance  length of the line in meters. Default 10.
#   world     default | city; falls back to the WORLD env var that start.sh exports, then `default`.
#             It must be the world the simulation runs.

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
YAW="${ARGS[0]:-0}"
DISTANCE="${ARGS[1]:-10}"
WORLD_NAME="${ARGS[2]:-$(basename "${WORLD:-default}" .world)}"

source /opt/ros/jazzy/setup.bash

if [[ $BUILD -eq 1 || ! -f install/setup.bash ]]; then
    colcon build --base-paths 3dpath --packages-select path3d
fi
source install/setup.bash

exec ros2 run path3d line_planner_demo "$YAW" "$DISTANCE" "$WORLD_NAME"
