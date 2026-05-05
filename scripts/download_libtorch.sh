#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."

VERSION="2.3.0"
INSTALL_DIR="lib/libtorch"

if [ -d "$INSTALL_DIR/include" ]; then
    echo "[OK] LibTorch $VERSION already installed at $INSTALL_DIR"
    exit 0
fi

echo "Downloading LibTorch $VERSION (CPU) ..."
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

curl -L "https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${VERSION}%2Bcpu.zip" -o "$TMPDIR/libtorch.zip"

mkdir -p "$INSTALL_DIR"
unzip -q "$TMPDIR/libtorch.zip" -d "$INSTALL_DIR"

# unzip creates libtorch/ subdirectory, move contents up
if [ -d "$INSTALL_DIR/libtorch" ]; then
    mv "$INSTALL_DIR/libtorch/"* "$INSTALL_DIR/"
    rmdir "$INSTALL_DIR/libtorch"
fi

echo "[OK] LibTorch $VERSION installed at $INSTALL_DIR"
