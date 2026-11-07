#!/usr/bin/env bash
set -e

SCRIPT=$(readlink -f "$0")
ROOT=$(dirname "$(dirname "$SCRIPT")")

# Container already running: open another shell in it instead of starting a second one
if [ -n "$(docker ps -q -f name=^3d-path-planner$)" ]; then
  exec docker exec -it 3d-path-planner bash
fi

# GPU detection: use the NVIDIA GPU only if it is actually usable, otherwise fall back to CPU
GPU_ARGS=()
if docker info 2>/dev/null | grep -qi "nvidia"; then
  # Linux with NVIDIA Container Toolkit installed
  GPU_ARGS=(--gpus all)
elif grep -qi microsoft /proc/version 2>/dev/null && command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1; then
  # WSL2 (Docker Desktop) with an NVIDIA Windows driver
  GPU_ARGS=(--gpus all)
fi

if [ ${#GPU_ARGS[@]} -gt 0 ]; then
  echo "NVIDIA GPU detected: enabling GPU rendering"
else
  echo "No usable NVIDIA GPU detected: using CPU rendering"
fi

# Resource limits, so the simulation + mapping cannot starve the host.
# Override: DOCKER_CPUS=6 DOCKER_MEMORY=10g ./docker/run.sh   (applies when the container starts)
# Default: two thirds of the cores and 8 GB of RAM. Memory-swap equals memory: no swapping, so a
# runaway process is killed inside the container instead of freezing the host.
DOCKER_CPUS="${DOCKER_CPUS:-$(( ($(nproc) * 2 + 2) / 3 ))}"
DOCKER_MEMORY="${DOCKER_MEMORY:-8g}"
echo "Limits: $DOCKER_CPUS CPUs, $DOCKER_MEMORY RAM"

exec docker run -it --rm --init \
  --cpus="$DOCKER_CPUS" \
  --memory="$DOCKER_MEMORY" --memory-swap="$DOCKER_MEMORY" \
  "${GPU_ARGS[@]}" \
  --name 3d-path-planner \
  --add-host=host.docker.internal:host-gateway \
  -p 8765:8765 \
  -v "$ROOT:/workspace" \
  -v "$(pwd)/docker/.tmux.conf:/root/.tmux.conf:ro" \
  3d-path-planner:dev bash