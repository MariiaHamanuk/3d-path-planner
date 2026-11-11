#!/bin/bash
# Builds path3d and runs path3d_main: depth camera -> log-odds occupancy grid -> Foxglove topics
# (/map/occupied_voxels, /map/occupied_cloud). Run it in the `planner` tmux window, after
# ./scripts/start.sh has brought up the simulation.
#
# Usage: run_mapping.sh [-n|--no-build] [world]
#   world: default | city; falls back to the WORLD env var that start.sh exports (a path or a
#   name), then to `default`. It must be the world the simulation runs: it names the TF frame.

set -e

ROOT=$(realpath "$(dirname "$(readlink -f "$0")")/..")
cd "$ROOT"

BUILD=1
WORLD_NAME=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        -n|--no-build) BUILD=0; shift ;;
        *) WORLD_NAME="$1"; shift ;;
    esac
done
if [[ -z "$WORLD_NAME" ]]; then
    WORLD_NAME=$(basename "${WORLD:-default}" .world)
fi

source /opt/ros/jazzy/setup.bash

if [[ $BUILD -eq 1 || ! -f install/setup.bash ]]; then
    colcon build --base-paths 3dpath --packages-select path3d
fi
source install/setup.bash

exec ros2 run path3d path3d_main "$WORLD_NAME"
