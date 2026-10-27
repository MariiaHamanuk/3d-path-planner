#!/bin/bash

set -x

SCRIPT=$(readlink -f $0)
ROOT=$(realpath "$(dirname "$SCRIPT")/../..")  
cd "$ROOT"

SCRIPTDIR="$ROOT/src/scripts"

# Usage: start.sh [-w|--world <name>] [config-path]    (name: default, city, ...; or WORLD=<name> env)
WORLD_ARG="$WORLD"
CONFIG=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        -w|--world) WORLD_ARG="$2"; shift 2 ;;
        *) CONFIG="$1"; shift ;;
    esac
done

if [[ -z "$CONFIG" ]]; then
    CONFIG="$SCRIPTDIR/config.cfg"
fi

# Load config
source "$CONFIG"
# precedence: -w / env WORLD > config WORLD > default
WORLD="${WORLD_ARG:-${WORLD:-default}}"
# bare name -> simulation/worlds/<name>.world
if [[ "$WORLD" != */* && "$WORLD" != *.* ]]; then
    WORLD="$ROOT/simulation/worlds/$WORLD.world"
fi
if [[ ! -f "$WORLD" ]]; then
    echo "World not found: $WORLD" >&2
    exit 1
fi
export WORLD

# Kill previous simulation instances
kill -9 $(pgrep -f 'gz sim') 2>/dev/null

# Start tmuxinator
tmuxinator start -p "$SCRIPTDIR/tmux.yml"