#!/usr/bin/env bash
# Cross-compile Tier-A (r3d_core, no ROS) for aarch64 (Jetson).
# Requires: gcc-aarch64-linux-gnu (host) + the aarch64 libc/libstdc++ sysroot
# (present under /usr/aarch64-linux-gnu on Ubuntu).
# GPIO backend is wiringPi (CMake option USE_WIRINGPI, default ON). A cross build
# usually lacks an aarch64 libwiringPi in the sysroot → CMake warns and gpio.cpp
# falls back to MOCK (logic-only; real GPIO must be built natively on the board).
# For real wiringPi here: install an aarch64 libwiringPi into the sysroot — or just
# rebuild natively on the device (recommended).
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
cd "$ROOT"

cmake -B build-aarch64 \
  -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.cmake \
  -DUSE_WIRINGPI=ON \
  "$@"
cmake --build build-aarch64 -j

echo "==== aarch64 产物 ===="
file build-aarch64/r3d_main build-aarch64/r3d_daemon build-aarch64/r3d_send_cmd
