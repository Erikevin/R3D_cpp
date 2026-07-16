# r3d_ws_bridge — 系统结构与实现说明

> `R3D/web_a.py`（Python，rclpy + asyncio + websockets）→ C++17 / ROS 2 Humble 移植。
> 同时附带移动端 Web 点云前端。架构遵循 `R3D_cpp/SYSTEM_STRUCTURE.md` 的蓝图
> （`web_a.py → ws_bridge_node.cpp`，**rclcpp + boost::beast**），代码风格对齐 `R3D_cpp`。

---

## 1. 这个项目是什么

`r3d_ws_bridge` 是 R3D 手持三维建图设备与手机 App 之间的 **WebSocket 桥接节点**：

- 订阅 LIO-SAM 的里程计 + 已配准点云（以及可选的相机压缩图）；
- 按时间戳把 **odom ↔ lidar** 配对同步；
- 在 `ws://0.0.0.0:8765/data` 提供 WebSocket 服务，向客户端推送 `img/odom/pcd`；
- 处理设备指令：查询存储/电量、开始/停止建图（通过 `gnome-terminal` + `xdotool` 启停 livox/lio_sam/相机脚本）、列出已保存点云文件夹；
- 跨进程指令经 `/tmp/shared_cmd.txt`（`{"cmd": 1|0}`）与 Tier-A 主控（`r3d_main`）通信，控制雷达上电。

它是一个标准的 **ament_cmake 包**，构建产出三个可执行文件：

| 可执行文件 | 作用 |
|---|---|
| `r3d_ws_bridge` | 桥接节点本体（= `web_a.py`） |
| `mock_publisher` | 无雷达环境下的假数据源（odom + pcd，可选图像） |
| `r3d_bench` | 热点路径微基准（parse / JSON / base64） |

---

## 2. 运行拓扑

```
            Livox MID-360 + LIO-SAM                  手机 / Web 客户端
        (/lio_sam/mapping/odometry)                      (WS client)
        (/lio_sam/mapping/cloud_registered)                  ^
        (/camera/image_raw/compressed)                       | ws://0.0.0.0:8765/data
                    ^                                        |
                    | BEST_EFFORT, keep_last(50)             |
            +-------+----------------------------------------+--------+
            |              r3d_ws_bridge (rclcpp 节点)                |
            |  · 订阅 odom/pcd/image                                  |
            |  · try_sync：按时间戳配对（newest-first）50ms 一次       |
            |  · boost::beast WS 服务端（单 io 线程，每会话读循环+推流）|
            |  · 指令分发：getStorage / startRecord / stopRecord /     |
            |    GET_FILE_LIST                                         |
            |  · statvfs 设备信息；gnome-terminal+xdotool 启停建图      |
            +-------+----------------------------------------+--------+
                    |                                        |
                    | /tmp/shared_cmd.txt {"cmd":1|0}        | gnome-terminal + xdotool
                    v                                        v
            Tier-A 主控 (r3d_main) → 雷达继电器      livox_driver2 / lio_sam / camera_two_value
                                                              |
                                                              v
                                              /home/jetson/Desktop/save_pcd/<ts>/  (pcd 文件夹)
```

两个进程**只**通过 `/tmp/shared_cmd.txt` 通信（`{"cmd": 1}` = 开雷达/开始建图，`{"cmd": 0}` = 关）。WS 客户端只连 `/data` 路径，其它路径直接关闭连接（对应 `web_a.py::router`）。

---

## 3. 模块映射 Python → C++

| `web_a.py`（Python） | C++ 文件 | 说明 |
|---|---|---|
| 全局配置常量 | `include/r3d_ws/types.hpp` | `constexpr`：`MAX_TIME_DIFF`、`PCD_SEND_INTERVAL`、`PCD_MAX_POINTS`、缓冲容量、`ENABLE_CAMERA_STREAM` |
| `parse_pointcloud2` | `types.hpp::parse_pointcloud2` | 按 `point_step` 遍历读 3 个 float，上限 8000 点（`memcpy` 替代 `struct.unpack`） |
| `SyncSubscriber`（订阅 + `try_sync`） | `ws_bridge_node.{hpp,cpp}` | rclcpp 节点：三个订阅 + 50ms 同步定时器 |
| `handle_command` | `ws_bridge_node.cpp::handle_command` | 指令分发（getStorage/startRecord/stopRecord/GET_FILE_LIST） |
| `handle_data`（receive_loop + send_loop） | `ws_server.{hpp,cpp}` | boost::beast 异步 WS：每会话读循环 + 50ms 推流 tick |
| `router`（路径 `/data`） | `ws_server.cpp`（HTTP 读 + `target()=="/data"` 判定） | 非 `/data` 直接关 socket |
| `send_device_info_on_connect` | `WsServer::ConnectHandler` | 连接建立即推 `deviceInfo` |
| `get_device_info` | `mapping_control.cpp::device_info` | `statvfs("/")` 算存储；电量/版本/设备号硬编码 |
| `open_terminal` / `start_mapping_commands` / `stop_process_in_terminal` / `close_all_windows` | `mapping_control.cpp` | 同样的 gnome-terminal + xdotool 序列与 sleep；headless 下优雅 no-op |
| `get_pcd_file_list` | `mapping_control.cpp::pcd_file_list` | 列 `save_pcd/` 子目录，按 ctime 倒序 |
| `write_shared_cmd` | `mapping_control.cpp::write_shared_cmd` | 原子写 `{"cmd": N}`（临时文件 + rename） |
| `run_ros2` + `__main__` | `src/main.cpp` | `rclcpp::init` + 信号处理 + `spin` |

---

## 4. 项目结构

```
ws_server/
├── SYSTEM_STRUCTURE.md            ← 本文件
├── README.md                      构建 / 运行 / 协议 / 部署
├── BENCHMARK.md                   Python vs C++ 性能对比
├── description.md                 原始需求
├── package.xml · CMakeLists.txt   ament_cmake 包定义
├── third_party/nlohmann/json.hpp  内置单头 JSON
├── include/r3d_ws/
│   ├── types.hpp        OdomData/LidarData/ImageData · parse_pointcloud2 · base64 · to_json
│   ├── mapping_control.hpp   设备信息 · gnome-terminal/xdotool · pcd 列表 · 跨进程指令
│   ├── ws_server.hpp    SharedState + boost::beast 异步 服务端/会话
│   ├── ws_bridge_node.hpp     rclcpp 节点（订阅 + 同步定时器 + 指令分发）
│   └── logger.hpp       包内日志器（对齐 R3D_cpp/r3d/logger.hpp）
├── src/
│   ├── ws_bridge_node.cpp · ws_server.cpp · mapping_control.cpp
│   ├── main.cpp         r3d_ws_bridge 入口（信号处理 + spin）
│   ├── mock_publisher.cpp   假 odom + pcd（可选 --camera）
│   └── bench.cpp        r3d_bench 微基准
├── tools/
│   ├── ws_client_test.py     纯标准库 WS 客户端（命令 / 流式验证）
│   └── bench/                基准测试套件 + 图表（见 BENCHMARK.md）
└── frontend/
    ├── index.html       移动优先仪表盘 · Three.js 渲染 · Demo 模式
    ├── three.min.js · OrbitControls.js   内置（r128，离线可用）
```

**静态库说明：** 核心代码编译成 **静态库** `r3d_ws_bridge_lib`（开启 PIC），再链接进三个可执行文件——避免运行期 `.so` 依赖与 RPATH 问题。

---

## 5. WebSocket 协议（与 `web_a.py` 完全一致）

客户端 → 服务端：
```json
{"cmd":"getStorage"} | {"cmd":"startRecord"} | {"cmd":"stopRecord"} | {"type":"GET_FILE_LIST"}
```
服务端 → 客户端：
```json
{"type":"deviceInfo","data":{"version","deviceCode","battery","storageTotal","storageUsed","storagePercent","wsStatus"}}
{"type":"img","data":"<base64 jpg>"}
{"type":"odom","data":{"t","x","y","z","qw","qx","qy","qz"}}
{"type":"pcd","data":{"t","points":[[x,y,z], …]}}
{"type":"response","cmd":"startRecord","status":"ok"}
{"code":200,"msg":"获取文件列表成功","data":[{"name","createTime","size":"文件夹"}, …]}
```
大帧（pcd ≈ 0.5 MB）服务端**关闭自动分片**（`auto_fragment(false)`），整帧发送；客户端需支持 126/127 扩展长度与续帧重组（`tools/ws_client_test.py` 已实现）。

---

## 6. 线程模型

```
主线程        ──► rclcpp::spin(node)          ROS 回调（img_cb/odom_cb/lidar_cb）+ try_sync 定时器
io 线程       ──► boost::asio io_context::run accept 循环 + 每会话读循环/推流 tick/串行写队列
后台线程(detach) ─► start_mapping_commands()   startRecord：sleep 5s 后开终端
后台线程(detach) ─► close_all_windows()        stopRecord：停进程 + 关窗口
```

- `SharedState`（mutex 守护的双端队列 + `latest_*` + 原子标志 `is_recording/mapping_ready`）由 ROS 回调写、WS 会话读。
- 单 io 线程 ⇒ 所有 asio 回调串行，天然无数据竞争；写操作统一过 `enqueue_write` 投递到会话串行队列，保证「指令响应 + 推流」不会并发写同一 stream（beast 允许「一读一写」并发）。
- `SIGINT/SIGTERM` → `rclcpp::shutdown()`，`spin` 返回，节点析构停止 WS 服务。

---

## 7. 时间同步（try_sync）——已修复陈旧问题

每 50 ms 执行：在 odom/lidar 缓冲区里找 `|t_o − t_l|` 最小且 `< MAX_TIME_DIFF(0.05s)` 的配对，写入 `latest_odom/latest_lidar`。

> **关键修复**：缓冲区按 **从新到旧** 遍历。因为 odom 与 lidar 每个时刻戳相同（差≈0，存在并列），用严格 `<` 做判定时，「先遇到的」就锁定结果——从新遍历就让**最新**配对胜出。
> 原 `web_a.py` 从旧遍历，会锁住缓冲区里**最旧**的配对，导致推出去的点云滞后约 2 秒（lidar 缓冲深 20 帧 ≈ 2s）。C++ 版修正后端到端延迟从 ~2000 ms 降到 **~77 ms(p50)**，吞吐不变。详见 BENCHMARK.md。

---

## 8. 构建与运行

```bash
source /opt/ros/humble/setup.bash
cd ws_server
colcon build --packages-select r3d_bringup
source install/setup.bash
```
依赖：`rclcpp`、`sensor_msgs`、`nav_msgs`、OpenCV（CMake find_package）、boost（beast/asio，仅头文件）、Threads。**不依赖 cv_bridge**（压缩图直接用 `cv::imdecode`）。

真机运行（有雷达 + 桌面）：
```bash
ros2 run r3d_bringup r3d_ws_bridge
```
headless 测试（无雷达、无显示器）：
```bash
ros2 run r3d_bringup mock_publisher &            # 造 odom+pcd@10Hz
R3D_FORCE_STREAM=1 ros2 run r3d_bringup r3d_ws_bridge &
```
`R3D_FORCE_STREAM=1`：推流循环无视 `is_recording/mapping_ready`，仅用于 headless 跑通数据通路（真机不用）。

---

## 9. 测试

- **命令通道**：`python3 tools/ws_client_test.py command 127.0.0.1 8765`
  → 校验 `deviceInfo` 推送、`getStorage`、`GET_FILE_LIST`。
- **数据通道**：跑 `mock_publisher` + `R3D_FORCE_STREAM=1` 的 bridge，再
  `python3 tools/ws_client_test.py stream 127.0.0.1 8765 3`
  → 校验 odom≈20Hz、pcd≈10Hz、8000 点/帧。
- **前端**：浏览器开 `frontend/index.html`，点「演示模式」验证渲染；或填 `ws://<IP>:8765/data` 连真实后端。

---

## 10. 性能基准（摘要）

详见 [BENCHMARK.md](BENCHMARK.md)。同机对比：

| 操作（每帧） | C++ | Python | C++ 倍率 |
|---|---:|---:|---:|
| PointCloud2 parse（8000 点） | ~11 µs | ~948 µs | **≈86×** |
| pcd JSON 序列化 | ~1.49 ms | ~6.24 ms | **≈4.2×**（且帧小 ~5%） |
| base64（100KB） | ~87 µs | ~72 µs | 0.83×（Python C 内置更快） |

端到端（C++）：pcd 9.7 Hz、吞吐 ~4.4 MB/s、CPU ~3%（单核）、峰值 RSS ~28 MB、延迟 ~77 ms(p50)。

> Python 端到端基准在本机跑不了（无 `pip`/`websockets`），需在装好该依赖的机器上用 `tools/bench/run_py_bridge.py`。

---

## 11. 与 Python 的差异（刻意为之）

1. **`startRecord`/`stopRecord`** 的阻塞工作（开终端、关窗口）放到后台线程，立即回 `ok`，不阻塞读循环——对客户端体验等价。
2. **图像解码**直接用 `cv::imdecode`/`cv::imencode`（无 cv_bridge）。
3. **`try_sync` 从新遍历**（见第 7 节）——修正 ~2s 陈旧，是有意改进，非忠实复刻。
4. **`R3D_FORCE_STREAM`** 调试环境变量，仅用于 headless 跑通数据通路。
5. **终端/窗口控制**在 headless 主机上优雅 no-op（记日志，不抛异常）。
6. WS 大帧**关闭自动分片**，整帧发送。
