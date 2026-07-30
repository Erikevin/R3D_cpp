# R3D_cpp

R3D 手持三维建图设备的 C++ 实现，分两层（架构全貌见 [SYSTEM_STRUCTURE.md](SYSTEM_STRUCTURE.md)）：

| 层 | 进程 | ROS | 构建 |
|----|------|-----|------|
| **Tier-A** 核心控制 | `r3d_main`（GPIO + UART + 状态机 + 监听 `/tmp/shared_cmd.txt`） | 无 | 顶层 CMake |
| **Tier-B** 数据/桥接 | `r3d_ws_bridge`、`image_odom_recorder`（ROS2 节点） | Humble | `ros2_ws/` colcon |

`r3d_main` 在运行时 fork 出 `r3d_ws_bridge`（桥在子进程里自己 source ROS）；手机下发 `startRecord` 后，桥再 fork 出 `livox` / `lio_sam` / 相机录制器（各自 source ROS + `/root/my_robot_d`）。**`r3d_main` 本体不依赖任何 ROS 环境。**

---

## 1. 依赖

**Tier-A（`r3d_main`）**
- C++17、CMake ≥ 3.13、pthreads
- GPIO 后端：`libwiringPi`（Orange Pi 上即 **wiringOP**）。找不到则 `gpio.cpp` 自动回退 MOCK 后端（仅日志，不操作真实引脚）——适合无硬件的开发机。
- UART：板子上的 `/dev/ttyS2`（`config.hpp` 可改）。

**Tier-B（`r3d_bringup`）**
- ROS2 Humble、colcon、OpenCV（相机录制器）
- 运行建图时还需 `/root/my_robot_d/install/setup.bash` 提供 `livox_ros_driver2` / `lio_sam`（仅运行依赖，**编译 `r3d_bringup` 本身不需要**）。

> GPIO 后端：`IGpio` 接口 + 两个编译期实现——`WiringPiGpio`（`USE_WIRINGPI=ON` 且找到 libwiringPi）与日志版 `MockGpio`（开发机无硬件、或交叉编译 sysroot 无库时自动回退）。用 `-DUSE_WIRINGPI=ON/OFF` 控制。

---

## 2. 编译

### 2.1 Tier-A：`r3d_main`（顶层 CMake）

```bash
cd R3D_cpp

# 开发机（无硬件）—— 找不到 libwiringPi 自动用 MOCK
cmake -B build && cmake --build build -j
./build/r3d_main

# 板子上（真实 GPIO）—— 先装 wiringOP，再开启 wiringPi 后端
sudo apt install -y wiringpi        # 或编译安装 wiringOP（Orange Pi 3B）
cmake -B build -DUSE_WIRINGPI=ON && cmake --build build -j
sudo ./build/r3d_main
```

交叉编译 aarch64（产物在 `build-aarch64/`，**GPIO 会回退 MOCK**，仅供逻辑验证，板子上仍需原生重编）：

```bash
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
bash scripts/cross-aarch64.sh        # 用 cmake/aarch64-linux-gnu.cmake 工具链
```

### 2.2 Tier-B：`r3d_bringup`（ROS2，`ros2_ws/`）

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select r3d_bringup
source install/setup.bash
ros2 run r3d_bringup r3d_ws_bridge        # exe 名是 r3d_ws_bridge（非 ws_bridge_node）
```

无雷达/无显示器的端到端联调（自带 mock 数据源）：`bash ros2_ws/src/r3d_bringup/tools/run_demo.sh`

---

## 3. 在 arm64 板子上部署安装（systemd 服务）

部署目录约定为 `/root/R3D_cpp`（与 `config.hpp` 的 `ROS_WS_DEFAULT=/root/R3D_cpp/ros2_ws` 一致）。`r3d_main` 用 [deploy/r3d-main.service](deploy/r3d-main.service) 做成系统服务，`Restart=on-failure` 替代旧的 `r3d_daemon`（后者靠 gnome-terminal，无头板已废）。

```bash
# 0) 板子上必须是 aarch64 原生二进制（开发机 build/ 里是 x86，不能用！）
file /root/R3D_cpp/build/r3d_main        # 应显示 ELF 64-bit LSB ... ARM aarch64
#   若不是 → 在板子上原生重编（见 §2.1 板子那段），勿直接拷 x86 产物。

# 1) 安装服务
sudo cp /root/R3D_cpp/deploy/r3d-main.service /etc/systemd/system/
sudo systemctl daemon-reload

# 2) 先停掉任何旧的自启方式，避免与 r3d-main 抢 GPIO / /dev/ttyS2 / 雷达
sudo systemctl stop r3d-daemon 2>/dev/null; sudo pkill -f r3d_daemon 2>/dev/null
#   若曾用 rc.local / Auto_start.py / ~/.bashrc 自启 r3d_daemon，一并移除。

# 3) 启用并启动（开机自启）
sudo systemctl enable --now r3d-main

# 4) 验证
systemctl status r3d-main
journalctl -u r3d-main -f                # r3d_main 日志（stderr → journald）
```

服务要点（详见文件头注释）：`User=root`；`WorkingDirectory=/root/R3D_cpp`（桥继承此 cwd 按相对路径读 `ros2_ws/src/r3d_bringup/config/mapping_commands.json`）；`ExecStart=/root/R3D_cpp/build/r3d_main`（板子上二进制路径不同就改这行）；本服务**只直接管 `r3d_main`**，桥与建图子进程经 `setsid` 脱离，但 `systemctl stop r3d-main` 时 `r3d_main` 的 SIGTERM 清理 + cgroup 兜底会把它们一并停掉。

可选：防硬件未接时每 3s 疯狂重启，在 `[Unit]` 后加 `StartLimitIntervalSec=60` / `StartLimitBurst=5`。

---

## 4. 调试

### 日志位置
| 来源 | 位置 |
|------|------|
| `r3d_main`（服务） | `journalctl -u r3d-main -f` |
| `r3d_ws_bridge`（桥） | `/tmp/r3d_bridge.log` |
| 建图子进程 livox / lio_sam / camera | `/tmp/r3d_mapping_{livox,lio_sam,camera}.log` |
| 跨进程指令 IPC（桥写、`r3d_main` 读） | `cat /tmp/shared_cmd.txt` → `{"cmd": 1}` 开雷达 / `0` 关 |

### 服务控制
```bash
systemctl status r3d-main          # 状态 + 最近日志
systemctl restart r3d-main         # 重启（崩了的桥/建图子进程也会被清掉再随主进程拉起）
systemctl stop r3d-main            # 优雅停（SIGTERM → r3d_main cleanup）
journalctl -u r3d-main --since "10 min ago"
ls -l /dev/ttyS2                   # 确认串口在；服务 Wants=dev-ttyS2.device
```

### 手动跑（绕过服务，便于加 gdb/printf）
```bash
sudo /root/R3D_cpp/build/r3d_main          # 前台，Ctrl+C 退出会走 cleanup
```

### WebSocket 命令/数据通道测试
```bash
# 桥默认监听 ws://0.0.0.0:8765/data
python3 ros2_ws/src/r3d_bringup/tools/ws_client_test.py command <ip> 8765   # 发指令
python3 ros2_ws/src/r3d_bringup/tools/ws_client_test.py stream   <ip> 8765 5 # 收 5s 点云
```

### 常见问题
- **服务起来了但 GPIO/LED/风扇不动** → 多半板子上没装 wiringPi，或用了交叉编译产物（GPIO 回退 MOCK）。`journalctl` 里搜 `Mock` / `未找到 wiringPi`。解决：板子原生编译 + 装 wiringOP。
- **桥报 `Package 'r3d_bringup' not found`** → `ros2_ws` 没编译或没 `source install/setup.bash`。先 `cd ros2_ws && colcon build --packages-select r3d_bringup`。
- **手机 startRecord 后建图没起** → 看 `/tmp/r3d_mapping_*.log`。最常见是 cwd 不对导致 `mapping_commands.json` 读不到而静默回退默认，或 `/root/my_robot_d/install/setup.bash` 不存在 → `source` 失败。确认 `r3d-main` 服务 `WorkingDirectory=/root/R3D_cpp`，且 `/root/my_robot_d` 已编译。
- **`r3d_main` 反复重启** → `journalctl -u r3d-main` 看崩溃点；常见是 `/dev/ttyS2` 权限或电源板未连接。
- **WS 端口占用 / `bind: Address already in use`** → `pkill -f r3d_ws_bridge` 清残留。

---

## 5. 相关文档
- [SYSTEM_STRUCTURE.md](SYSTEM_STRUCTURE.md) — 系统架构、Python→C++ 对照、保真度说明
- [ros2_ws/src/r3d_bringup/README.md](ros2_ws/src/r3d_bringup/README.md) / [WS_BRIDGE.md](ros2_ws/src/r3d_bringup/WS_BRIDGE.md) — 桥与录制器实现细节
- [deploy/r3d-main.service](deploy/r3d-main.service) — `r3d_main` systemd 服务模板
