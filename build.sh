#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

# Check system dependencies
OS=$(uname -s)
if [ "$OS" = "Linux" ]; then
    MISSING=""
    for pkg in cmake g++ curl libglfw3-dev libyaml-cpp-dev libtbb-dev; do
        if ! dpkg -s "$pkg" &>/dev/null; then
            MISSING="$MISSING $pkg"
        fi
    done
    if [ -n "$MISSING" ]; then
        echo "Missing system packages:$MISSING"
        echo "Install with: sudo apt-get install$MISSING"
        exit 1
    fi
fi

# Download dependencies if missing
bash scripts/download_mujoco.sh
bash scripts/download_onnxruntime.sh

echo ""
echo "Building..."
cmake -B build -DCMAKE_BUILD_TYPE=Release
NPROC=$(sysctl -n hw.ncpu 2>/dev/null || nproc)
cmake --build build -j${NPROC}

echo ""
echo "Done. Run: ./run.sh"
