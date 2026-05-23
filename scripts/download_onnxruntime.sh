#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."

VERSION="1.19.2"
INSTALL_DIR="lib/onnxruntime"

if [ -d "$INSTALL_DIR/include/onnxruntime" ] || [ -f "$INSTALL_DIR/include/onnxruntime_c_api.h" ]; then
    echo "[OK] ONNX Runtime $VERSION already installed at $INSTALL_DIR"
    exit 0
fi

echo "Downloading ONNX Runtime $VERSION ..."
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

ARCH=$(uname -m)
OS=$(uname -s)
if [ "$OS" = "Darwin" ]; then
    if [ "$ARCH" = "arm64" ]; then
        PLATFORM="osx-arm64"
    else
        PLATFORM="osx-x86_64"
    fi
elif [ "$OS" = "Linux" ]; then
    if [ "$ARCH" = "x86_64" ]; then
        PLATFORM="linux-x64"
    elif [ "$ARCH" = "aarch64" ]; then
        PLATFORM="linux-aarch64"
    else
        echo "Unsupported Linux architecture: $ARCH"
        exit 1
    fi
else
    echo "Unsupported OS: $OS"
    exit 1
fi

curl -L "https://github.com/microsoft/onnxruntime/releases/download/v${VERSION}/onnxruntime-${PLATFORM}-${VERSION}.tgz" -o "$TMPDIR/onnxruntime.tgz"

mkdir -p "$INSTALL_DIR"
tar xzf "$TMPDIR/onnxruntime.tgz" -C "$INSTALL_DIR" --strip-components=1

echo "[OK] ONNX Runtime $VERSION installed at $INSTALL_DIR"
