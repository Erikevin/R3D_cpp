#pragma once
// r3d_bringup (Tier-B) configuration — centralized constants for the WebSocket
// bridge, grouped by function. Mirrors the r3d/config.hpp convention of Tier-A:
// source files reference r3d_ws::config::XXX rather than hardcoding values.

#include <cstddef>

namespace r3d_ws::config {

// ---- Camera stream ---------------------------------------------------------
inline constexpr bool ENABLE_CAMERA_STREAM = false;  // True=stream frames, False=off
inline constexpr int  SEND_WIDTH           = 960;
inline constexpr int  SEND_HEIGHT          = 540;

// ---- Stream timing ---------------------------------------------------------
inline constexpr double MAX_TIME_DIFF     = 0.05;  // s — odom/lidar sync window
inline constexpr double PCD_SEND_INTERVAL = 0.2;      // s — 5 Hz pcd cap
inline constexpr int    SEND_TICK_INTERVAL_MS = 200;  // ms — 5 Hz send tick (odom/img)

// ---- Buffers ---------------------------------------------------------------
inline constexpr std::size_t PCD_MAX_POINTS   = 8000;
inline constexpr std::size_t ODOM_BUFFER_CAP  = 50;
inline constexpr std::size_t LIDAR_BUFFER_CAP = 20;

// ---- Network ---------------------------------------------------------------
inline constexpr unsigned short WS_PORT = 8765;

// ---- Paths -----------------------------------------------------------------
inline constexpr const char* SAVE_PCD_DIR = "/home/jetson/Desktop/save_pcd/";
inline constexpr const char* SHARE_FILE   = "/tmp/shared_cmd.txt";
// Telemetry published by r3d_main (battery/voltage/temp). Bridge reads battery
// from here in deviceInfo. /tmp is tmpfs → memory-only, no disk contention.
inline constexpr const char* TELEMETRY_FILE = "/tmp/shared_telemetry.txt";

// ---- Mapping command configuration (externalized JSON) ---------------------
// JSON holding the three background-process command strings for
// MappingControl::start_mapping_commands(). Resolved RELATIVE TO THE BRIDGE CWD
// (parent of ros2_ws — same assumption the command strings themselves rely on,
// which reference "ros2_ws/install/setup.bash"). Override with an absolute path
// via $R3D_MAPPING_CMD_FILE when that assumption does not hold.
inline constexpr const char* MAPPING_CMD_FILE_ENV     = "R3D_MAPPING_CMD_FILE";
inline constexpr const char* MAPPING_CMD_FILE_DEFAULT = "ros2_ws/src/r3d_bringup/config/mapping_commands.json";

// ---- startRecord 前置相机整定（adjust_cam_param 快速档） ---------------------
// 收到 startRecord 后先整定左右相机曝光/增益（cam_tuner.hpp，≤2s、≤5轮、双机
// 并行），完成后才拉起建图进程链。设备须与 image_odom_recorder 的
// camera_device0/camera_device2 参数一致（默认 /dev/video0 与 /dev/video2）。
inline constexpr bool TUNE_ENABLE = true;   // false=跳过整定，直接按原流程启动
inline constexpr const char* TUNE_DEV0 = "/dev/video0";  // 左相机（camera0）
inline constexpr const char* TUNE_DEV2 = "/dev/video2";  // 右相机（camera1）

} // namespace r3d_ws::config
