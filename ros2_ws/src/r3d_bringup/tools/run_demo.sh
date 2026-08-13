#!/usr/bin/env bash
# run_demo.sh — 一键启动 headless 数据通路（无雷达、无显示器也能跑通）。
#   流程：清理残留 → mock_publisher + r3d_ws_bridge(R3D_FORCE_STREAM=1)
#        → 3 秒自检 → 前台保活（Ctrl+C 自动清理子进程）
#
# 用法：  ./tools/run_demo.sh  （在 r3d_bringup 包目录或任意位置均可）
# 退出：  Ctrl+C（会自动 kill mock + bridge）
#
# 说明：  mock 与 bridge 的日志分别写到 /tmp/r3d_mock.log、/tmp/r3d_bridge.log
#         本脚本位于 ros2_ws/src/r3d_bringup/tools/，colcon 产物在 ros2_ws/install/
set -o pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"      # r3d_bringup 包目录
WS="$(cd "$ROOT/../.." && pwd)"     # ros2_ws 工作区根（colcon 产物在 $WS/install）
cd "$WS"

# ---- ROS 环境 ----
# 注意：不能加 `set -u`，ROS 的 setup.bash 会引用未定义变量。
source /opt/ros/humble/setup.bash 2>/dev/null
source install/setup.bash 2>/dev/null

# 包名 r3d_bringup；可执行名仍为 r3d_ws_bridge / mock_publisher。
BIN="install/r3d_bringup/lib/r3d_bringup"
MOCK_PID=""; BRIDGE_PID=""; CLEANING=0

cleanup() {
  [ "$CLEANING" = 1 ] && return
  CLEANING=1
  echo
  echo "[run_demo] 正在停止服务..."
  [ -n "$BRIDGE_PID" ] && kill -INT "$BRIDGE_PID" 2>/dev/null
  [ -n "$MOCK_PID" ]   && kill -INT "$MOCK_PID"   2>/dev/null
  sleep 1
  pkill -9 -f 'r3d_bringup/lib/r3d_bringup' 2>/dev/null
  pkill -9 -f 'mock_publisher' 2>/dev/null
  echo "[run_demo] 已清理。"
}
trap cleanup EXIT

# ---- 1) 先清理可能的残留进程（避免 bind: Address already in use） ----
echo "[run_demo] 清理残留进程..."
pkill -INT -f 'r3d_bringup/lib/r3d_bringup' 2>/dev/null
pkill -INT -f 'mock_publisher' 2>/dev/null
sleep 1
pkill -9 -f 'r3d_bringup/lib/r3d_bringup' 2>/dev/null
pkill -9 -f 'mock_publisher' 2>/dev/null

# ---- 2) 检查是否已编译 ----
if [ ! -x "$BIN/r3d_ws_bridge" ] || [ ! -x "$BIN/mock_publisher" ]; then
  echo "[run_demo] ❌ 未找到可执行文件，请先在 ros2_ws 下编译："
  echo "    source /opt/ros/humble/setup.bash && colcon build --packages-select r3d_bringup"
  exit 1
fi

# ---- 3) 启动 mock_publisher ----
echo "[run_demo] 启动 mock_publisher（odom + pcd @10Hz）..."
"$BIN/mock_publisher" > /tmp/r3d_mock.log 2>&1 &
MOCK_PID=$!
sleep 0.5

# ---- 4) 启动 bridge（强制推流） ----
echo "[run_demo] 启动 r3d_ws_bridge (R3D_FORCE_STREAM=1)..."
R3D_FORCE_STREAM=1 "$BIN/r3d_ws_bridge" >> /tmp/r3d_bridge.log 2>&1 &
BRIDGE_PID=$!

# ---- 5) 等待 WS 端口绑定（探测 8765 监听，不依赖日志内容 → append 友好）----
echo "[run_demo] 等待 WS 服务就绪..."
ok=0
for _ in $(seq 1 50); do
  if (exec 3<>"/dev/tcp/127.0.0.1/8765") 2>/dev/null; then exec 3>&- 3<&-; ok=1; break; fi
  sleep 0.1
done
if [ "$ok" = 0 ]; then
  echo "[run_demo] ❌ bridge 启动失败，日志尾部："
  tail -n 10 /tmp/r3d_bridge.log
  exit 1
fi
sleep 1.5   # 等 DDS 发现 + 缓冲区填充
echo "[run_demo] ✅ 服务已就绪。"

# ---- 6) 自检：3 秒数据通路 ----
echo "[run_demo] 自检（3s 数据通路）..."
python3 "$ROOT/tools/ws_client_test.py" stream 127.0.0.1 8765 3 \
  && echo "[run_demo] ✅ 自检通过" \
  || echo "[run_demo] ⚠️ 自检未通过（可能 mock 未被发现，稍等再试 ws_client_test）"

# ---- 7) 打印后续操作 ----
cat <<EOF

============================================================
[run_demo] 数据通路已就绪，服务在前台保活（Ctrl+C 退出并清理）

  WS 地址       : ws://127.0.0.1:8765/data
  bridge 日志   : tail -f /tmp/r3d_bridge.log
  mock 日志     : tail -f /tmp/r3d_mock.log

  命令通道测试  : python3 $ROOT/tools/ws_client_test.py command 127.0.0.1 8765
  持续收数据    : python3 $ROOT/tools/ws_client_test.py stream 127.0.0.1 8765 10
  性能基准      : bash $ROOT/tools/bench/run_benchmark.sh 8
  前端          : 浏览器打开 $ROOT/frontend/index.html
                   → 连接 ws://localhost:8765/data
                   → 或点「演示模式」（无需后端）
============================================================
EOF

# ---- 8) 前台保活，直到 Ctrl+C（EXIT trap 会清理）----
wait
