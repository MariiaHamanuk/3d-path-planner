#!/bin/bash
# Builds path3d and runs the smooth planner demo: Lazy Theta* + SmoothLocalPlanner fly the drone to a goal.
# The drone must already be flying in GUIDED mode: start the simulation (./scripts/start.sh), take
# off (QGroundControl's Takeoff, or in the `ardupilot` window: `mode guided`, `arm throttle`,
# `takeoff 5`), then run this in the `planner` tmux window.
#
# Usage: run_smooth_planner.sh [-n|--no-build] <x> <y> <z> [speed] [world]
#   x y z     goal in the world frame (ENU: x east, y north, z up), m
#   speed     cruise speed, m/s. Default 5.
#   world     default | city; falls back to the WORLD env var that start.sh exports, then `default`.

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
if [[ ${#ARGS[@]} -lt 3 ]]; then
    echo "usage: run_smooth_planner.sh [-n|--no-build] <x> <y> <z> [speed] [world]" >&2
    exit 2
fi
SPEED="${ARGS[3]:-5}"
WORLD_NAME="${ARGS[4]:-$(basename "${WORLD:-default}" .world)}"

source /opt/ros/jazzy/setup.bash

if [[ $BUILD -eq 1 || ! -f install/setup.bash ]]; then
    colcon build --base-paths 3dpath --packages-select path3d
fi
source install/setup.bash

exec ros2 run path3d smooth_planner_demo "${ARGS[0]}" "${ARGS[1]}" "${ARGS[2]}" "$SPEED" "$WORLD_NAME"
