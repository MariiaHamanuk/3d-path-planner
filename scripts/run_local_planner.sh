#!/bin/bash
# Builds path3d and runs the local planner demo: the drone flies to a goal relative to where it is,
# around the obstacles it maps on the way (A* or Lazy Theta* + the path-following local planner).
# The drone must already be flying in GUIDED mode: start the simulation (./scripts/start.sh), take
# off (QGroundControl's Takeoff, or in the `ardupilot` window: `mode guided`, `arm throttle`,
# `takeoff 5`), then run this in the `planner` tmux window.
#
# Usage: run_local_planner.sh [-n|--no-build] [dx] [dy] [dz] [world] [planner]
#   dx dy dz  the goal relative to the drone, in meters (x east, y north, z up). Default 20 0 0.
#   world     default | city; falls back to the WORLD env var that start.sh exports, then `default`.
#             It must be the world the simulation runs.
#   planner   astar | theta (Lazy Theta*). Default astar.

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
DX="${ARGS[0]:-20}"
DY="${ARGS[1]:-0}"
DZ="${ARGS[2]:-0}"
WORLD_NAME="${ARGS[3]:-$(basename "${WORLD:-default}" .world)}"
PLANNER="${ARGS[4]:-astar}"

source /opt/ros/jazzy/setup.bash

if [[ $BUILD -eq 1 || ! -f install/setup.bash ]]; then
    colcon build --base-paths 3dpath --packages-select path3d
fi
source install/setup.bash

exec ros2 run path3d local_planner_demo "$DX" "$DY" "$DZ" "$WORLD_NAME" "$PLANNER"
