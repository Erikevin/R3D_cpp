#!/usr/bin/env bash
# Cross-compile Tier-A (r3d_core, no ROS) for aarch64 (Jetson).
# Requires: gcc-aarch64-linux-gnu (host) + the aarch64 libc/libstdc++ sysroot
# (present under /usr/aarch64-linux-gnu on Ubuntu).
# For the LIBGPIOD backend also: sudo dpkg --add-architecture aarch64 &&
#   sudo apt update && sudo apt install libgpiod-dev:aarch64
# (otherwise gpio.cpp falls back to MOCK via __has_include).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT"

cmake -B build-aarch64 \
  -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake \
  -DR3D_GPIO_BACKEND="${R3D_GPIO_BACKEND:-LIBGPIOD}" \
  "$@"
cmake --build build-aarch64 -j

echo "==== aarch64 产物 ===="
file build-aarch64/r3d_main build-aarch64/r3d_daemon build-aarch64/r3d_send_cmd
