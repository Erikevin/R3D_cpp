# r3d_ws_bridge — C++ port of `R3D/web_a.py` + mobile pointcloud frontend

A faithful, runtime-equivalent C++17 / ROS 2 (Humble) realization of
[`R3D/web_a.py`](../R3D/web_a.py) — the WebSocket bridge that sits between the R3D
hand-held mapping device and its mobile app — plus a standalone, mobile-first web
page that renders the live (or simulated) point cloud.

The architecture follows `R3D_cpp/SYSTEM_STRUCTURE.md` (which maps
`web_a.py → ws_bridge_node.cpp`, **rclcpp + boost::beast**). Conventions match the
`R3D_cpp` codebase (namespaces, logger, `{"cmd":N}` IPC, signal handling).

> Efficiency: the C++ port is **~86×** faster at parsing point clouds and **~4.2×**
> faster at JSON than the Python original. See [BENCHMARK.md](BENCHMARK.md).
> Architecture & module map: [SYSTEM_STRUCTURE.md](SYSTEM_STRUCTURE.md).

---

## What it does

Mirrors `web_a.py` exactly:

- Subscribes (BEST_EFFORT, keep-last 50) to
  `/camera/image_raw/compressed`, `/lio_sam/mapping/odometry`,
  `/lio_sam/mapping/cloud_registered`.
- Every 50 ms, time-syncs odometry ↔ lidar by closest stamp (`MAX_TIME_DIFF = 0.05 s`).
- Serves a WebSocket on **`ws://0.0.0.0:8765/data`** (other paths are rejected).
- On connect: pushes `deviceInfo`.
- Commands: `getStorage`, `startRecord`, `stopRecord`, `GET_FILE_LIST`.
- Streams `img` / `odom` / `pcd` while recording (pcd rate-limited to 10 Hz).

---

## Layout

```
ws_server/
├── package.xml · CMakeLists.txt        ament_cmake package: r3d_ws_bridge
├── third_party/nlohmann/json.hpp       vendored single-header JSON
├── include/r3d_ws/
│   ├── types.hpp        OdomData/LidarData/ImageData, parse_pointcloud2, base64, to_json
│   ├── mapping_control.hpp   device info · gnome-terminal/xdotool · pcd list · shared cmd
│   ├── ws_server.hpp    SharedState + boost::beast async server/session
│   ├── ws_bridge_node.hpp     rclcpp node (subs + sync timer + command dispatch)
│   └── logger.hpp       package-local logger (mirrors R3D_cpp/r3d/logger.hpp)
├── src/
│   ├── ws_bridge_node.cpp · ws_server.cpp · mapping_control.cpp
│   ├── main.cpp         r3d_ws_bridge entry (signal handlers + spin)
│   ├── mock_publisher.cpp   fake PointCloud2 + Odometry (+ optional --camera)
│   └── bench.cpp        r3d_bench microbenchmark
├── tools/
│   ├── ws_client_test.py     stdlib-only WS client (command / stream verify)
│   └── bench/                benchmark suite + chart (see BENCHMARK.md)
└── frontend/
    ├── index.html       mobile-first dashboard, Three.js viewer, Demo mode
    ├── three.min.js · OrbitControls.js   vendored (r128, offline)
```

---

## Build

```bash
source /opt/ros/humble/setup.bash
cd ws_server
colcon build --packages-select r3d_bringup
# executables → build/r3d_bringup/{r3d_ws_bridge,mock_publisher,r3d_bench}
```

Dependencies: `rclcpp`, `sensor_msgs`, `nav_msgs`, OpenCV (found via CMake), boost
(beast/asio — header-only), Threads. **No cv_bridge** (compressed images are
decoded with `cv::imdecode` directly).

## Run

```bash
source install/setup.bash
ros2 run r3d_bringup r3d_ws_bridge        # or: install/r3d_bringup/lib/r3d_bringup/r3d_ws_bridge
```

Open the frontend (any static host, or just the file) on a phone:
`frontend/index.html` → enter `ws://<device-ip>:8765/data` → **Connect**.

### Headless testing (no LiDAR, no display)

One command — cleans leftovers, starts mock + bridge (`R3D_FORCE_STREAM=1`),
self-checks the data path, and stays in the foreground (Ctrl+C tears down):

```bash
./tools/run_demo.sh
```

…or do it manually:

```bash
ros2 run r3d_bringup mock_publisher &            # publishes fake odom + pcd @10 Hz
R3D_FORCE_STREAM=1 ros2 run r3d_bringup r3d_ws_bridge &
python3 tools/ws_client_test.py stream 127.0.0.1 8765 3   # expect odom+pcd frames
```

`R3D_FORCE_STREAM=1` makes the send loop stream whenever data is present, ignoring
`is_recording`/`mapping_ready` — purely so the data path can be exercised on a
machine without `gnome-terminal`/`xdotool`. Inert when unset. Add `--camera` to the
mock to also publish a synthetic `CompressedImage`.

---

## image_odom_recorder — C++ port of `R3D/camera/camera_two_value.py`

Dual-camera V4L2 native-MJPG recorder, time-aligned to the LiDAR. Sits in this
package alongside `ws_bridge_node` (per `SYSTEM_STRUCTURE.md` §5/§6).

- A **trigger thread** watches the Livox mmap timestamp (`~/timeshare`); on each
  new ns (≥ `1/fps` apart) it wakes **both** independent camera threads, which grab
  the latest MJPG frame and save it under the **same** LiDAR-ns filename
  (`images/camera0|1/<sec>.jpg`) → frames align with `/livox/*` and `odom.txt`.
- Raw MJPG bytes are written straight to disk by an async IO thread (zero
  decode/encode); `odom.txt` records `/lio_sam/mapping/odometry`.
- Optionally forks `ros2 bag record` into `<output_dir>/rosbag2` for
  `/livox/imu /livox/lidar` (params `record_bag`, `bag_topics`).

```bash
ros2 run r3d_bringup image_odom_recorder
ros2 launch r3d_bringup recorder.launch.py
```

Params: `odom_topic, output_dir, camera_device0|2, image_width, image_height, fps,
jpeg_quality, max_pending, record_bag, bag_topics`. Source files: `src/capture_node.cpp`
+ `v4l2_camera.cpp` / `odom_writer.cpp` / `ts_reader.cpp`, headers under
`include/r3d_camodom/` (namespace `r3d_camodom`).

---

## WebSocket protocol (identical to `web_a.py`)

Client → server:
```json
{"cmd":"getStorage"} | {"cmd":"startRecord"} | {"cmd":"stopRecord"} | {"type":"GET_FILE_LIST"}
```

Server → client:
```json
{"type":"deviceInfo","data":{"version","deviceCode","battery","storageTotal","storageUsed","storagePercent","wsStatus"}}
{"type":"img","data":"<base64 jpg>"}
{"type":"odom","data":{"t","x","y","z","qw","qx","qy","qz"}}
{"type":"pcd","data":{"t","points":[[x,y,z], …]}}
{"type":"response","cmd":"startRecord","status":"ok"}
{"code":200,"msg":"获取文件列表成功","data":[{"name","createTime","size":"文件夹"}, …]}
```

---

## Frontend

`frontend/index.html` — single file, no build step, works offline (Three.js vendored).

- **Connect** to `ws://<device-ip>:8765/data`, or **Demo Mode** to render a synthetic
  rotating point cloud + moving odometry with no device (satisfies "simulate and render").
- 3D viewport: `THREE.Points` colored by height, odometry marker + traveled-path
  trail, touch-friendly orbit controls. ROS axes (x-fwd, y-left, z-up) mapped to
  three.js (z-up).
- Dashboard: battery / version / storage bar, start/stop record, file list, telemetry,
  pcd frame rate. Auto-reconnect with backoff.

---

## Fidelity notes (intentional differences from Python)

- `startRecord`/`stopRecord` run their blocking work (terminal launches, window
  cleanup) on background threads and reply immediately, instead of blocking the
  receive loop for seconds. Same UX for the client.
- Image decode uses `cv::imdecode`/`cv::imencode` directly (no `cv_bridge`).
- `R3D_FORCE_STREAM` debug env added to exercise the data path headlessly.
- Terminal/window control no-ops gracefully on headless hosts (logs, never throws).
- **`try_sync` iterates newest-first** (unlike `web_a.py`, which iterates oldest-first
  and ships ~2 s stale data). This is an intentional improvement: end-to-end pcd
  latency drops from ~2 000 ms to **~77 ms (p50)** with no throughput change. See
  [BENCHMARK.md](BENCHMARK.md).
