#!/bin/bash
set -e
cd "$(dirname "${BASH_SOURCE[0]}")/.."

VERSION="3.2.7"
INSTALL_DIR="lib/mujoco"

if [ -d "$INSTALL_DIR/include/mujoco" ]; then
    echo "[OK] MuJoCo $VERSION already installed at $INSTALL_DIR"
    exit 0
fi

echo "Downloading MuJoCo $VERSION ..."
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

curl -L "https://github.com/google-deepmind/mujoco/releases/download/${VERSION}/mujoco-${VERSION}-linux-x86_64.tar.gz" -o "$TMPDIR/mujoco.tar.gz"

mkdir -p "$INSTALL_DIR"
tar xzf "$TMPDIR/mujoco.tar.gz" -C "$INSTALL_DIR" --strip-components=1

echo "[OK] MuJoCo $VERSION installed at $INSTALL_DIR"
