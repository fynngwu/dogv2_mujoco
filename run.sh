#!/bin/bash
cd "$(dirname "${BASH_SOURCE[0]}")"
exec ./build/bin/dogv2_mujoco "$@"
