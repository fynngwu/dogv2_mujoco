#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

# Download dependencies if missing
bash scripts/download_mujoco.sh
bash scripts/download_libtorch.sh

echo ""
echo "Building..."
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

echo ""
echo "Done. Run: ./run.sh"
