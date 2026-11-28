#!/bin/bash
# Usage: run_all_planners.sh [runs]

SCRIPT=$(readlink -f "$0")
ROOT=$(realpath "$(dirname "$SCRIPT")/..")
cd "$ROOT"

SCRIPTDIR="$ROOT/scripts"
RUNS="${1:-1}"
ALGOS=(astar theta dijkstra rrt rrt_star rrt_connect)

source "$SCRIPTDIR/config.cfg"
WORLD_NAME=$(basename "${WORLD:-default}" .world)
WORLD="${WORLD:-default}"
if [[ "$WORLD" != */* && "$WORLD" != *.* ]]; then
    WORLD="$ROOT/simulation/worlds/$WORLD.world"
fi
export WORLD

stop_sim() {
    tmux kill-session -t simulation 2>/dev/null || true
    pkill -9 -f 'gz sim|arducopter|sim_vehicle.py|mavproxy|parameter_bridge' 2>/dev/null || true
    sleep 2
}

wait_and_takeoff() {
    local t=0
    while [[ $t -lt 90 ]]; do
        tmux capture-pane -t simulation:ardupilot -p -S -200 2>/dev/null | grep -Eq 'EKF[23].*using GPS' && break
        sleep 2; t=$((t + 2))
    done

    tmux send-keys -t simulation:ardupilot "mode guided" C-m
    sleep 2

    t=0
    while [[ $t -lt 60 ]]; do
        tmux send-keys -t simulation:ardupilot "arm throttle" C-m
        sleep 3
        tmux capture-pane -t simulation:ardupilot -p -S -30 2>/dev/null | grep -Eq 'ARMED|Arming motors' && break
        t=$((t + 3))
    done

    tmux send-keys -t simulation:ardupilot "takeoff 5" C-m
    sleep 10
}

trap stop_sim EXIT INT TERM

source /opt/ros/jazzy/setup.bash
colcon build --base-paths 3dpath --packages-select path3d

for ALGO in "${ALGOS[@]}"; do
    for ((RUN = 1; RUN <= RUNS; RUN++)); do
        echo "=== $ALGO ($RUN/$RUNS) ==="
        stop_sim
        tmuxinator start -p "$SCRIPTDIR/tmux.yml" --no-attach
        wait_and_takeoff

        DONE="/tmp/path3d_done_$$"
        rm -f "$DONE"
        tmux send-keys -t simulation:planner \
            "timeout 300 ./scripts/run_local_planner.sh -n -p --no-viz --no-log $X $Y $Z $WORLD_NAME $ALGO; echo \$? > $DONE" C-m

        while [[ ! -f "$DONE" ]]; do sleep 2; done
        echo "=== $ALGO ($RUN/$RUNS) exit=$(cat "$DONE") ==="
        rm -f "$DONE"
        stop_sim
    done
done
