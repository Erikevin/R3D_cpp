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
inline constexpr double PCD_SEND_INTERVAL = 0.1;   // s — 10 Hz pcd cap

// ---- Buffers ---------------------------------------------------------------
inline constexpr std::size_t PCD_MAX_POINTS   = 8000;
inline constexpr std::size_t ODOM_BUFFER_CAP  = 50;
inline constexpr std::size_t LIDAR_BUFFER_CAP = 20;

// ---- Network ---------------------------------------------------------------
inline constexpr unsigned short WS_PORT = 8765;

// ---- Paths -----------------------------------------------------------------
inline constexpr const char* SAVE_PCD_DIR = "/home/jetson/Desktop/save_pcd/";
inline constexpr const char* SHARE_FILE   = "/tmp/shared_cmd.txt";

} // namespace r3d_ws::config
