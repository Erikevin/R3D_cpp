# R3D — System Structure & C++ Realization

> Translation blueprint for the `R3D` Python codebase → C++17.
> Source under `R3D/`, C++ realization under `R3D_cpp/`.

---

## 1. What R3D Is

R3D is a **battery-powered, hand-held 3D-mapping device** built on an NVIDIA Jetson
(runs on `/dev/ttyTHS1` — the Tegra HSU UART, and `/dev/video0|2`). It fuses:

- a **Livox MID-360 LiDAR** running **LIO-SAM** SLAM (ROS 2 Humble),
- **two synchronized USB cameras** stamped with the LiDAR hardware timestamp,
- a **custom power-management board** spoken to over UART,
- a **mobile/web client** that connects over **WebSocket** to start/stop mapping and
  stream odometry + point-cloud + (optionally) video.

The Python tree is organised into three runtime tiers:

| Tier | Files | Depends on | Runs as |
|------|-------|------------|---------|
| **A. Core control** | `main.py, system.py, hardware.py, serial_protocol.py, protection.py, config.py, daemon.py` | GPIO + UART only (no ROS) | `r3d_main` (+ `r3d_daemon` watchdog) |
| **B. ROS 2 nodes** | `web.py, web_a.py, video_c.py, camera/camera*.py` | rclpy, OpenCV, websockets | ROS 2 nodes (mapping session) |
| **C. Boot + tools** | `Auto_start.py, text.sh, uart_tool.py, 1.py, 2.py, test_process.py` | shell / Tkinter | boot script + dev GUIs |

---

## 2. Runtime Topology

```
                       Power board                Mobile/Web App
                      (MCU over UART)                 (WS client)
                            ^                              ^
       0x5C 0x31 shutdown   |     0x5C 0x34 radar on/off   |  ws://x.x.x.x:8765/data
                            |                              |
            +---------------+--------------+---------------+-----------------+
            |        r3d_core (main)       |        r3d_ws_bridge (ROS node) |
            |  - GPIO LEDs / scan blink    |  - subscribes odom/pcd/image     |
            |  - UART rx -> shutdown flow  |  - time-sync odom<->lidar         |
            |  - state machine INIT/RUN/.. |  - start/stop mapping            |
            |  - IPC monitor (/tmp/cmd)    |  - spawns ROS launch in terminals|
            +---------------+--------------+---------------+-----------------+
                            |                              |
                            |   /tmp/shared_cmd.txt        | xdotool/gnome-terminal
                            |   {"cmd": 1|0}               | launches:
                            v                              v
                   radar power relay        livox_driver2 / lio_sam / r3d_camera
                                                     |
                                                     v
                                          /home/<u>/timeshare  (16-byte mmap,
                                           int64 high, int64 low = Livox ns)
                                                     |
                                                     v
                                    r3d_camera_recorder stamps each frame &
                                    writes JPEG + odom.txt under save_pcd/<ts>/
```

Two processes talk **only** through the file `/tmp/shared_cmd.txt` (`{"cmd": 1}` =
radar ON / start mapping, `{"cmd": 0}` = OFF / stop). The Livox driver writes its
current timestamp into a 16-byte mmap file `~/timeshare`; the camera node reads it
so every photo is stamped with the **real LiDAR time** (post-processing alignment).

---

## 3. Wire Protocol (power board)

Fixed **9-byte frames**: `[0x5C][CMD][payload×5][0x0D][0x0A]`. All parsing/encoding
lives in `serial_protocol.cpp` (`SerialProtocol::parse_buffer` / `send`).

### 3.1 Power board → Jetson — `0x31` cyclic telemetry (sent continuously)

```
[5C][31][U_H][U_L][Temp][Power_off][Battery][0D][0A]
  0   1   2    3     4       5          6       7 8
```

| Field        | Byte (off) | Meaning |
|--------------|------------|---------|
| U_H / U_L    | 2, 3       | Voltage, big-endian, in **mV** → logged as V (`(U_H<<8)|U_L / 1000`) |
| Temp         | 4          | Enclosure temperature, °C (raw byte) |
| Power_off    | 5          | **>0** ⇒ board requests shutdown |
| Battery      | 6          | Battery level 0–100 (%) |

Every received chunk is shown as raw hex, then each `0x31` frame is decoded
(no throttling — every frame is printed):
```
[HH:MM:SS.mmm][INFO] [串口] ← 收到(9B): 5C 31 31 20 23 00 57 0D 0A
[HH:MM:SS.mmm][INFO] [串口] 遥测: 电压=12.58V  温度=35  电量=87%  关机请求=否
```
All log lines carry **millisecond** timestamps (`[HH:MM:SS.mmm]`, see `logger.hpp`).

### 3.2 Jetson → power board — commands

| CMD  | Meaning |
|------|---------|
| 0x32 | Fan on/off — byte3 `>0` on / `=0` off → `FAN_ON` / `FAN_OFF` |
| 0x33 | Allow shutdown — byte3 `=1` → `ALLOW_SHUTDOWN` |
| 0x34 | Radar power — byte3 `=1` on / `=0` off → `RADAR_ON` / `RADAR_OFF` |

Command strings carry embedded `0x00`, so they are built with the
`std::string(ptr, len)` constructor — the plain `string(const char*)` form would
truncate at the first null (e.g. `FAN_ON` would shrink 9→3 bytes). Every `send()`
also logs its bytes: `[串口] → 发送: 5C 32 01 00 00 00 00 0D 0A`.

### 3.3 Auto behaviors driven by the telemetry frame

- **Auto fan (thermal):** when `Temp > FAN_TEMP_THRESHOLD` (default **45 °C**,
  adjustable in `config.hpp`) the Jetson sends `FAN_ON`; when it drops back to
  ≤ threshold it sends `FAN_OFF`. **Edge-triggered** (one command per threshold
  crossing) so the cyclic report never spams fan commands.
- **Shutdown triggers** — either fires the same flow, **latched once per run**:
  1. `Power_off > 0` — board explicitly requests shutdown.
  2. `Battery < BAT_LOW` (< 10 %) — low battery.

  Flow: log the reason → send `ALLOW_SHUTDOWN` → wait `ALLOW_SHUTDOWN_DELAY_MS`
  (0.5 s) → invoke the shutdown callback (`System::shutdown()`).

---

## 4. State Machine (`system.py`)

```
        init_all() ok?                       recv 0x5C31 shutdown
   INIT ───────yes──────► RUN ──────start_radar──────► SCAN
     │                    │  ◄─────stop_radar─────────┘
     │ no                 │ camera offline / fault
     └──────────────────► ERROR ◄─────────────────────┘
                          │   battery < BAT_PROTECT
                          └────► LOW_BAT ──► SHUTDOWN
LED mapping: RUN=green, ERROR/LOW_BAT=red, SCAN=green blinking @1 Hz.
```

---

## 5. Module Map — Python → C++

| Python                     | C++ file                                | Notes |
|----------------------------|-----------------------------------------|-------|
| `config.py`                | `include/r3d/config.hpp`                | `constexpr` constants |
| `hardware.py` (RPi.GPIO)   | `include/r3d/gpio.hpp` + `src/gpio.cpp`  | `IGpio` + wiringPi/mock (compile-time via `USE_WIRINGPI`) |
| `hardware.py` (blink/leds) | `include/r3d/hardware.hpp` + `.cpp`      | `std::thread` blink loop |
| `serial_protocol.py`       | `serial.hpp` + `serial_protocol.hpp/.cpp`| POSIX termios UART |
| `protection.py`            | `protection.hpp` + `.cpp`                | battery monitor stub |
| `main.py` IPC monitor      | `ipc.hpp` + `.cpp` (SharedCmd)           | file-based IPC, minimal JSON |
| `system.py`                | `system.hpp` + `.cpp`                    | state machine |
| `main.py` entry            | `src/main.cpp`                           | + signal handlers |
| `daemon.py`                | `src/daemon.cpp`                         | respawn watchdog |
| `test_process.py`          | `src/send_cmd.cpp`                       | CLI IPC writer |
| `web_a.py`                 | `ros2_ws/.../ws_bridge_node.cpp`         | rclcpp + boost::beast WS |
| `camera/camera_two_value.py` | `ros2_ws/.../image_odom_recorder.cpp`  | rclcpp + OpenCV, dual cam |
| `video_c.py`               | `ros2_ws/.../usb_camera_node.cpp`        | rclcpp image publisher |
| `uart_tool.py`, `1.py`, `2.py` | `tools/README.md`                  | Tkinter GUIs → noted, Qt port optional |
| `Auto_start.py`, `text.sh` | `scripts/r3d_autostart.sh`               | shell boot wrapper (unchanged logic) |

---

## 6. C++ Project Layout

```
R3D_cpp/
├── SYSTEM_STRUCTURE.md            ← this file
├── README.md                      ← build / run / deploy
├── CMakeLists.txt                 ← builds the Tier-A core (no ROS)
├── include/r3d/
│   ├── config.hpp
│   ├── gpio.hpp                   ← IGpio + LibgpiodGpio / SysfsGpio / MockGpio
│   ├── hardware.hpp
│   ├── serial.hpp
│   ├── serial_protocol.hpp
│   ├── protection.hpp
│   ├── ipc.hpp                    ← SharedCmd file IPC
│   ├── logger.hpp
│   └── system.hpp
├── src/
│   ├── gpio.cpp
│   ├── hardware.cpp
│   ├── serial.cpp
│   ├── serial_protocol.cpp
│   ├── protection.cpp
│   ├── ipc.cpp
│   ├── system.cpp
│   ├── main.cpp                   ← r3d_main
│   ├── daemon.cpp                 ← r3d_daemon
│   └── send_cmd.cpp               ← r3d_send_cmd
├── scripts/
│   └── r3d_autostart.sh
└── ros2_ws/src/r3d_bringup/       ← Tier-B (separate ament_cmake package)
    ├── CMakeLists.txt
    ├── package.xml
    └── src/
        ├── ws_bridge_node.cpp
        ├── image_odom_recorder.cpp
        └── usb_camera_node.cpp
```

**Tier split matters for build deps:**

- **Tier A (`R3D_cpp/`)** — links only **pthread** + optional **libwiringPi**. Compiles
  and runs (mock GPIO) on any Linux for development; deploys to the Jetson for real.
- **Tier B (`ros2_ws/`)** — a standard ROS 2 package needing `rclcpp`, `sensor_msgs`,
  `nav_msgs`, `OpenCV`, `cv_bridge`, and `boost` (for the WebSocket server). Built
  inside a sourced ROS 2 workspace with `colcon`.

---

## 7. Dependency & Backend Choices

| Concern            | Python              | C++ choice                  | Why |
|--------------------|---------------------|-----------------------------|-----|
| GPIO               | RPi.GPIO (BOARD)    | **wiringPi** (+ mock)       | `IGpio` interface; wiringPi pin numbering (Orange Pi) |
| UART               | pyserial            | **POSIX termios**           | zero dependency |
| Threading          | threading           | **std::thread / mutex / atomic** | standard |
| Async I/O (WS)     | asyncio + websockets| **boost::asio + boost::beast** | already a ROS 2 transitive dep |
| Image / camera     | cv2                 | **OpenCV C++ (V4L2)**       | same backend |
| ROS 2              | rclpy               | **rclcpp**                  | canonical C++ client |
| IPC                | JSON file           | hand parser (`{"cmd":N}`)   | file is tiny & fixed-shape |
| Shared-mem ts      | mmap + struct       | **mmap + memcpy int64**     | identical binary layout |

GPIO is behind an `IGpio` interface with two compile-time backends: `WiringPiGpio`
(when `USE_WIRINGPI=ON` **and** libwiringPi is found) and the logging `MockGpio`
otherwise. `MockGpio` lets the whole core build & run on a dev PC with logging
instead of hardware access.

---

## 8. Threading Model (Tier A)

```
main thread ────────────► System::start_main_flow()   (debug console / state loop)
IPC thread   ──daemon──► monitor /tmp/shared_cmd.txt  → start/stop radar
UART rx thread (SerialProtocol) ─────────────────────► detect 0x5C31 → shutdown
blink thread  (Hardware, while is_scanning) ─────────► 1 Hz green toggle
protection thread (Protection) ──────────────────────► 1 s tick (battery stub)
```

Signals `SIGINT`/`SIGTERM` set a global `std::atomic<bool> g_running`; the main loop
checks it and `finally`-style cleanup turns LEDs off and closes the UART.

---

## 9. Build & Run (summary — full detail in README.md)

```bash
# Tier A — core control (dev PC, mock GPIO)
cd R3D_cpp && cmake -B build -DUSE_WIRINGPI=OFF && cmake --build build -j   # MOCK（无硬件）
./build/r3d_main

# Tier A — on the Jetson (real hardware)
cmake -B build -DUSE_WIRINGPI=ON && cmake --build build -j                # 需 libwiringPi 已装
sudo ./build/r3d_main

# Tier B — ROS 2 nodes
cd ros2_ws && colcon build --packages-select r3d_bringup
source install/setup.bash
ros2 run r3d_bringup ws_bridge_node      # WS server :8765/data
ros2 run r3d_bringup image_odom_recorder # dual-cam + odom recorder
```

---

## 10. Fidelity Notes (intentional differences from Python)

1. **GPIO**: RPi.GPIO does not actually run on the board; the C++ uses **wiringPi**
   (Orange Pi's wiringOP), with pin numbers in wiringPi numbering in `config.hpp`
   (verify with `gpio readall` against your board).
2. **`shutdown()`** in Python calls `subprocess.run(["echo","123","|","sudo",...])`
   which is broken (echo isn't a pipe). The C++ uses `execlp("shutdown","-h","now")`
   after dropping to root / via a polkit/sudo helper — correct and safe.
3. **WebSocket**: the Python `is_recording && mapping_ready` send-loop becomes a
   `std::thread` per connection pushing from the synced buffers under a mutex.
4. **GUI tools** (`uart_tool.py`, bag viewers `1.py`/`2.py`) are dev utilities not
   used at runtime on the device; they are documented for a future Qt port rather
   than translated now, to keep the realization focused on the device firmware.
5. **Camera auto-WB** variant (`camera_two_value.py`) is folded into the recorder via
   a `auto_wb` ROS parameter — one node, not three near-duplicate files.
