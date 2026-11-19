#!/bin/bash
# Builds path3d and runs the smooth planner demo: Lazy Theta* + SmoothLocalPlanner fly the drone to a goal.
# The drone must already be flying in GUIDED mode: start the simulation (./scripts/start.sh), take
# off (QGroundControl's Takeoff, or in the `ardupilot` window: `mode guided`, `arm throttle`,
# `takeoff 5`), then run this in the `planner` tmux window.
#
# Usage: run_smooth_planner.sh [-n|--no-build] [-c|--config <path>] [<x> <y> <z> [speed] [world] [algo]]
#   Without arguments everything comes from the config (default scripts/config.cfg):
#     DST="x y z"   goal in the world frame (ENU: x east, y north, z up), m
#     ALGO          lazy_theta_star (default) | astar | dijkstra | rrt | rrt_star | rrt_connect
#     SPEED         cruise speed, m/s. Default 5.
#     WORLD         default | city; the WORLD env var that start.sh exports wins, then `default`.
#   Arguments override the config.

set -e

ROOT=$(realpath "$(dirname "$(readlink -f "$0")")/..")
cd "$ROOT"

BUILD=1
CONFIG="scripts/config.cfg"
ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--no-build) BUILD=0; shift ;;
        -c|--config) CONFIG="$2"; shift 2 ;;
        *) ARGS+=("$1"); shift ;;
    esac
done

ENV_WORLD="$WORLD"
source "$CONFIG"
read -r -a DST_XYZ <<< "$DST"
X="${ARGS[0]:-${DST_XYZ[0]}}"
Y="${ARGS[1]:-${DST_XYZ[1]}}"
Z="${ARGS[2]:-${DST_XYZ[2]}}"
if [[ -z "$X" || -z "$Y" || -z "$Z" ]]; then
    echo "no goal: set DST=\"x y z\" in $CONFIG or pass <x> <y> <z>" >&2
    exit 2
fi
SPEED="${ARGS[3]:-${SPEED:-5}}"
WORLD_NAME="${ARGS[4]:-$(basename "${ENV_WORLD:-${WORLD:-default}}" .world)}"
ALGO="${ARGS[5]:-${ALGO:-lazy_theta_star}}"

source /opt/ros/jazzy/setup.bash

if [[ $BUILD -eq 1 || ! -f install/setup.bash ]]; then
    colcon build --base-paths 3dpath --packages-select path3d
fi
source install/setup.bash

exec ros2 run path3d smooth_planner_demo "$X" "$Y" "$Z" "$SPEED" "$WORLD_NAME" "$ALGO"
