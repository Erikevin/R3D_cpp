#pragma once
// R3D global configuration — C++ mirror of R3D/config.py
// All values are compile-time constants; adjust here for the target board.
// Constants are grouped by function — source files reference config::XXX rather
// than hardcoding magic numbers.

#include <cstdint>
#include <string>

namespace r3d::config {

// ---- Debug -----------------------------------------------------------------
// Skip camera detection (mirrors config.DEBUG_SKIP_CAMERA).
inline constexpr bool DEBUG_SKIP_CAMERA = true;

// ---- UART ------------------------------------------------------------------
inline const std::string UART_PORT   = "/dev/ttyS2";
inline constexpr int      UART_BAUD  = 9600;
inline constexpr int      UART_TIMEOUT_SEC = 1;   // logical timeout (not raw read)

// ---- GPIO (wiringPi numbering, Orange Pi) ---------------------------------
// Verify against the target board with `gpio readall`.
inline constexpr int GREEN_LED = 1;   // wiringPi 1 — also used as the scan/blink pin
inline constexpr int RED_LED   = 0;   // wiringPi 0
inline constexpr int FAN_PIN   = 2;   // wiringPi 2 (reserved; not yet initialized)
inline constexpr int PWM_SCAN  = 1;   // same physical pin as GREEN_LED

// ---- System timing ---------------------------------------------------------
inline constexpr int BOOT_WAIT_SEC           = 2;
inline constexpr int INIT_TIMEOUT            = 60;   // (reserved for future init guard)
inline constexpr int SHUTDOWN_DELAY_MS       = 1000; // (reserved for future shutdown pause)
inline constexpr int BLINK_HALF_PERIOD_MS    = 500;  // scan blink @1 Hz (half period)
inline constexpr int IPC_POLL_MS             = 100;  // shared_cmd file poll interval
inline constexpr int MAIN_LOOP_IDLE_MS       = 100;  // main state-loop idle sleep
inline constexpr int CAMERA_ERROR_RECHECK_MS = 200;  // recheck after camera-offline
inline constexpr int RADAR_RESEND_SEC        = 1;    // RADAR_ON/OFF sent twice, this apart
inline constexpr int SERIAL_RECV_POLL_MS     = 10;   // uart recv loop poll
inline constexpr int ALLOW_SHUTDOWN_DELAY_MS  = 500;  // pause after final ALLOW_SHUTDOWN → callback
inline constexpr int ALLOW_SHUTDOWN_RETRIES   = 3;    // low-battery: send ALLOW_SHUTDOWN N times (guard against dropped frames)
inline constexpr int ALLOW_SHUTDOWN_RETRY_SEC = 1;    // interval between repeated ALLOW_SHUTDOWN sends
inline constexpr int SHUTDOWN_GRACE_SEC       = 15;   // max wait for mapping to stop cleanly before power-off
inline constexpr int PROTECTION_HEARTBEAT_SEC = 1;   // protection monitor tick
inline constexpr int DAEMON_RESTART_SEC      = 3;    // respawn wait after r3d_main crash

// ---- Protection thresholds -------------------------------------------------
inline constexpr int BAT_LOW     = 10;
inline constexpr int BAT_PROTECT = 5;
inline constexpr int FAN_TEMP_THRESHOLD = 45;  // °C — auto fan on when enclosure temp exceeds this

// ---- Protocol frame (power board) -----------------------------------------
// All frames are 9 bytes: [0x5C][cmd][payload×5][0x0D][0x0A].
inline constexpr std::uint8_t FRAME_HEADER = 0x5C;
inline constexpr int          FRAME_LEN    = 9;
inline const std::string      FRAME_END    = std::string("\x0D\x0A", 2);  // bytes 8,9 (0x0D 0x0A)

// 0x31 = power-board→main cyclic telemetry report (sent continuously). Field
// offsets are relative to the 0x5C header at index 0.
inline constexpr std::uint8_t TELEMETRY_CMD            = 0x31;
inline constexpr int          TELEMETRY_VOLT_H_OFF     = 2;    // voltage high byte
inline constexpr int          TELEMETRY_VOLT_L_OFF     = 3;    // voltage low byte (V=(H<<8)|L, unit TBD)
inline constexpr int          TELEMETRY_TEMP_OFF       = 4;    // enclosure temperature
inline constexpr int          TELEMETRY_POWER_OFF_OFF  = 5;    // >0 ⇒ board requests shutdown
inline constexpr int          TELEMETRY_BATT_OFF       = 6;    // battery level 0-100 (%)

// NOTE: must use the (const char*, count) constructor — the bytes contain
// embedded 0x00, so the plain string(const char*) form would truncate at the
// first null (e.g. FAN_ON would shrink from 9 to 3 bytes: "5C 32 01").
inline const std::string FAN_ON         = std::string("\x5C\x32\x01\x00\x00\x00\x00\x0D\x0A", 9);
inline const std::string FAN_OFF        = std::string("\x5C\x32\x00\x00\x00\x00\x00\x0D\x0A", 9);
inline const std::string ALLOW_SHUTDOWN = std::string("\x5C\x33\x01\x00\x00\x00\x00\x0D\x0A", 9);
inline const std::string RADAR_ON       = std::string("\x5C\x34\x01\x00\x00\x00\x00\x0D\x0A", 9);
inline const std::string RADAR_OFF      = std::string("\x5C\x34\x00\x00\x00\x00\x00\x0D\x0A", 9);

// ---- IPC -------------------------------------------------------------------
// Shared command file (bridge writes, r3d_main reads). Holds {"cmd": N}.
inline const std::string SHARE_FILE = "/tmp/shared_cmd.txt";
// Telemetry file (r3d_main writes, bridge reads) so deviceInfo reports the real
// battery level instead of a hardcoded value. Holds {"battery":N,"voltage":V,"temp":T}.
// /tmp is tmpfs → memory-only I/O, never contends with rosbag2/PCD disk writes.
inline const std::string TELEMETRY_FILE = "/tmp/shared_telemetry.txt";
inline constexpr int     CMD_RADAR_ON  = 1;   // open radar / start mapping
inline constexpr int     CMD_RADAR_OFF = 0;   // close radar / stop mapping

// ---- System paths / launcher ----------------------------------------------
inline constexpr const char* MAIN_BIN_ENV     = "R3D_MAIN_BIN";  // override r3d_main path
inline constexpr const char* MAIN_BIN_DEFAULT = "r3d_main";
inline constexpr const char* TERMINAL_CMD     = "gnome-terminal"; // daemon + bridge launcher
inline constexpr const char* VIDEO_DEV_0      = "/dev/video0";
inline constexpr const char* VIDEO_DEV_2      = "/dev/video2";
inline constexpr const char* SHUTDOWN_BIN     = "shutdown";      // execlp("shutdown","-h","now")

// Bridge (r3d_bringup) launch — mirrors R3D/Auto_start.py: open a gnome-terminal
// that sources ROS + the workspace and runs the bridge node. The ROS workspace
// is overridable via $R3D_ROS_WS (dev box vs Jetson deploy path).
inline constexpr const char* ROS_WS_ENV     = "R3D_ROS_WS";
inline constexpr const char* ROS_WS_DEFAULT = "/root/R3D_cpp/ros2_ws";
inline constexpr const char* ROS_SETUP      = "/opt/ros/humble/setup.bash";
inline constexpr const char* BRIDGE_PACKAGE = "r3d_bringup";
inline constexpr const char* BRIDGE_NODE    = "r3d_ws_bridge";
inline constexpr const char* BRIDGE_LOG     = "/tmp/r3d_bridge.log";  // headless-mode stdout/stderr

// ---- System state ----------------------------------------------------------
enum class State {
    Init,
    Run,
    Scan,
    Error,
    LowBattery,
    Shutdown,
};

const char* state_name(State s);

} // namespace r3d::config
