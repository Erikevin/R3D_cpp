#!/usr/bin/env bash
# Reproducible Python-vs-C++ benchmark for the WS bridge.
#   Tier 1: microbenchmarks (parse_pointcloud2 / JSON / base64) — both impls, stdlib-only Python.
#   Tier 2: end-to-end WS load (C++; Python needs `websockets`, see BENCHMARK.md).
# Usage: ./run_benchmark.sh [LOAD_SECONDS]
set -o pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"     # r3d_bringup 包目录
WS="$(cd "$ROOT/../.." && pwd)"       # ros2_ws 工作区根（colcon 产物在 $WS/install）
cd "$WS"

source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null
BIN=install/r3d_bringup/lib/r3d_bringup
DURATION="${1:-8}"

cleanup() { pkill -INT -f r3d_bringup/lib 2>/dev/null || true; pkill -f mock_publisher 2>/dev/null || true; sleep 0.6; pkill -9 -f r3d_bringup/lib 2>/dev/null || true; }
trap cleanup EXIT

echo "########## TIER 1 — microbenchmarks ##########"
echo "### C++ (r3d_bench) ###"
"$BIN/r3d_bench"
echo
echo "### Python (bench_parse.py) ###"
python3 "$HERE/bench_parse.py"

echo
echo "########## TIER 2 — end-to-end WS load (${DURATION}s) ##########"
echo "### C++ (r3d_ws_bridge + mock_publisher) ###"
"$BIN/mock_publisher" >/tmp/mock.log 2>&1 &
MOCK=$!
sleep 0.5
R3D_FORCE_STREAM=1 "$BIN/r3d_ws_bridge" >/tmp/bridge.log 2>&1 &
BRIDGE=$!
for _ in $(seq 1 50); do grep -q "ws://0.0.0.0:8765" /tmp/bridge.log && break; sleep 0.1; done
sleep 2  # let DDS discover + buffers fill

CLK_TCK=$(getconf CLK_TCK)
cpu1=$(awk '{print $14+$15}' /proc/$BRIDGE/stat 2>/dev/null || echo 0)
python3 "$HERE/bench_server.py" 127.0.0.1 8765 "$DURATION"
cpu2=$(awk '{print $14+$15}' /proc/$BRIDGE/stat 2>/dev/null || echo 0)
hwm=$(awk '/VmHWM/{print $2}' /proc/$BRIDGE/status 2>/dev/null || echo 0)  # kB
echo "bridge_cpu_percent_1core=$(awk "BEGIN{printf \"%.1f\", ($cpu2-$cpu1)/$DURATION/$CLK_TCK*100}")"
echo "bridge_peak_rss_mb=$(awk "BEGIN{printf \"%.1f\", $hwm/1024}")"

echo
echo "### Python (web_a.py) ###"
if python3 -c "import websockets" 2>/dev/null; then
  "$HERE/run_py_bridge.py" >/tmp/pybridge.log 2>&1 &
  PYB=$!
  sleep 2
  python3 "$HERE/bench_server.py" 127.0.0.1 8765 "$DURATION"
  hwm=$(awk '/VmHWM/{print $2}' /proc/$PYB/status 2>/dev/null || echo 0)
  echo "py_peak_rss_mb=$(awk "BEGIN{printf \"%.1f\", $hwm/1024}")"
  kill -INT $PYB 2>/dev/null || true
else
  echo "python_skipped=websockets_not_installed"
fi
