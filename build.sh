#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
if [[ "${1:-}" == "--run" ]]; then
    exec ./build/campus_path_engine
fi
echo "Build termine. Lancement : ./build/campus_path_engine"
