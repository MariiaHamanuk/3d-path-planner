#!/bin/bash

set -x

SCRIPT=$(readlink -f $0)
ROOT=$(realpath "$(dirname "$SCRIPT")/../..")  
cd "$ROOT"

SCRIPTDIR="$ROOT/src/scripts"

CONFIG="$1"

if [[ -z "$CONFIG" ]]; then
    CONFIG="$SCRIPTDIR/config.cfg"
fi

# Load config
source "$CONFIG"
: "${WORLD:=$ROOT/simulation/worlds/default.world}"
export WORLD

# Kill previous simulation instances
kill -9 $(pgrep -f 'gz sim') 2>/dev/null

# Start tmuxinator
tmuxinator start -p "$SCRIPTDIR/tmux.yml"