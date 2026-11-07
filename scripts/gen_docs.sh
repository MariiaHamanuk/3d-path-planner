#!/bin/bash
# Generates the API reference (docs/api/index.html) from the doxygen comments in 3dpath/include.
# Uses the local doxygen if there is one; otherwise runs in a throwaway container (which installs
# doxygen first if the image predates it; docker/Dockerfile now includes it).
set -e
cd "$(realpath "$(dirname "$(readlink -f "$0")")/..")"

if command -v doxygen >/dev/null; then
    doxygen Doxyfile
else
    docker run --rm -v "$PWD":/ws -w /ws 3d-path-planner:dev bash -c "
        command -v doxygen >/dev/null || { apt-get update -qq && apt-get install -y -qq --no-install-recommends doxygen graphviz; } >/dev/null
        doxygen Doxyfile
        chown -R $(id -u):$(id -g) docs/api"
fi
echo "Open docs/api/index.html"
