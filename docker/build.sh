#!/usr/bin/env bash
set -e

SCRIPT=$(readlink -f $0)
ROOT=$(dirname "$(dirname "$SCRIPT")")

docker build -t 3d-path-planner:dev -f "$ROOT/docker/Dockerfile" "$ROOT"