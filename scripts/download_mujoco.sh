#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."

VERSION="3.8.1"
INSTALL_DIR="lib/mujoco"

if [ -d "$INSTALL_DIR/include/mujoco" ]; then
    echo "[OK] MuJoCo $VERSION already installed at $INSTALL_DIR"
    exit 0
fi

echo "Downloading MuJoCo $VERSION ..."
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

ARCH=$(uname -m)
OS=$(uname -s)

mkdir -p "$INSTALL_DIR"

if [ "$OS" = "Darwin" ]; then
    curl -L "https://github.com/google-deepmind/mujoco/releases/download/${VERSION}/mujoco-${VERSION}-macos-universal2.dmg" -o "$TMPDIR/mujoco.dmg"
    hdiutil attach "$TMPDIR/mujoco.dmg" -nobrowse -mountpoint "$TMPDIR/mnt" > /dev/null 2>&1
    # Extract headers and lib from macOS framework layout
    FW="$TMPDIR/mnt/mujoco.framework/Versions/A"
    mkdir -p "$INSTALL_DIR/include/mujoco" "$INSTALL_DIR/lib"
    cp "$FW/Headers/"* "$INSTALL_DIR/include/mujoco/"
    cp "$FW/libmujoco.${VERSION}.dylib" "$INSTALL_DIR/lib/"
    # Create symlinks for versioned dylib
    ln -sf "libmujoco.${VERSION}.dylib" "$INSTALL_DIR/lib/libmujoco.3.dylib"
    ln -sf "libmujoco.${VERSION}.dylib" "$INSTALL_DIR/lib/libmujoco.dylib"
    hdiutil detach "$TMPDIR/mnt" -quiet

    # Fix install_name from framework path to flat @rpath
    install_name_tool -id "@rpath/libmujoco.${VERSION}.dylib" "$INSTALL_DIR/lib/libmujoco.${VERSION}.dylib"
    codesign -f -s - "$INSTALL_DIR/lib/libmujoco.${VERSION}.dylib"
elif [ "$OS" = "Linux" ]; then
    if [ "$ARCH" = "x86_64" ]; then
        PLATFORM="linux-x86_64"
    elif [ "$ARCH" = "aarch64" ]; then
        PLATFORM="linux-aarch64"
    else
        echo "Unsupported Linux architecture: $ARCH"
        exit 1
    fi
    curl -L "https://github.com/google-deepmind/mujoco/releases/download/${VERSION}/mujoco-${VERSION}-${PLATFORM}.tar.gz" -o "$TMPDIR/mujoco.tar.gz"
    tar xzf "$TMPDIR/mujoco.tar.gz" -C "$INSTALL_DIR" --strip-components=1
else
    echo "Unsupported OS: $OS"
    exit 1
fi

echo "[OK] MuJoCo $VERSION installed at $INSTALL_DIR"
