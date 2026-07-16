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
// termios raw-read polling: VMIN=0/VTIME=1 → 0.1 s poll so the rx loop can spin.
inline constexpr int      UART_VMIN  = 0;
inline constexpr int      UART_VTIME = 1;

// ---- GPIO (BOARD numbering, as in config.py) -------------------------------
inline constexpr int GREEN_LED = 19;   // also used as the scan/blink pin
inline constexpr int RED_LED   = 21;
inline constexpr int FAN_PIN   = 2;
inline constexpr int PWM_SCAN  = 19;   // same physical pin as GREEN_LED

// BOARD pin -> libgpiod line number map.
// !! Board-specific: verify against your Jetson model's gpio line names
//    (`gpioinfo`). The values below are placeholders for Orin Nano dev kit.
struct PinMap { int board; const char* chip; unsigned line; };
inline constexpr PinMap GPIO_MAP[] = {
    {19, "gpiochip0", 79},   // GREEN_LED / PWM_SCAN
    {21, "gpiochip0", 14},   // RED_LED
    { 2, "gpiochip0", 70},   // FAN_PIN
};

// ---- GPIO sysfs / libgpiod details ----------------------------------------
inline constexpr const char* SYSFS_GPIO_BASE       = "/sys/class/gpio";
inline constexpr const char* GPIO_CONSUMER         = "r3d";  // libgpiod line consumer label
inline constexpr int         GPIO_EXPORT_SETTLE_MS = 100;    // settle wait after sysfs export

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
inline constexpr int ALLOW_SHUTDOWN_DELAY_MS = 500;  // pause after ALLOW_SHUTDOWN → callback
inline constexpr int PROTECTION_HEARTBEAT_SEC = 1;   // protection monitor tick
inline constexpr int DAEMON_RESTART_SEC      = 3;    // respawn wait after r3d_main crash

// ---- Protection thresholds -------------------------------------------------
inline constexpr int BAT_LOW     = 10;
inline constexpr int BAT_PROTECT = 5;

// ---- Protocol frame (power board) -----------------------------------------
inline constexpr std::uint8_t FRAME_HEADER           = 0x5C;
inline constexpr std::uint8_t SHUTDOWN_CMD_BYTE      = 0x31; // board→Jetson shutdown request
inline constexpr int          SHUTDOWN_PAYLOAD_OFFSET = 5;   // payload byte that matters
inline constexpr std::uint8_t SHUTDOWN_PAYLOAD_VALUE  = 1;   // data[offset]==1 → shutdown
inline const std::string      FRAME_END = std::string("\x0D\x0A", 2);

inline const std::string FAN_ON         = "\x5C\x32\x01\x00\x00\x00\x00\x0D\x0A";
inline const std::string FAN_OFF        = "\x5C\x32\x00\x00\x00\x00\x00\x0D\x0A";
inline const std::string ALLOW_SHUTDOWN = "\x5C\x33\x01\x00\x00\x00\x00\x0D\x0A";
inline const std::string RADAR_ON       = "\x5C\x34\x01\x00\x00\x00\x00\x0D\x0A";
inline const std::string RADAR_OFF      = "\x5C\x34\x00\x00\x00\x00\x00\x0D\x0A";

// ---- IPC -------------------------------------------------------------------
// Shared command file (bridge writes, r3d_main reads). Holds {"cmd": N}.
inline const std::string SHARE_FILE = "/tmp/shared_cmd.txt";
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
inline constexpr const char* ROS_WS_DEFAULT = "/home/jetson/R3D_cpp/ros2_ws";
inline constexpr const char* ROS_SETUP      = "/opt/ros/humble/setup.bash";
inline constexpr const char* BRIDGE_PACKAGE = "r3d_bringup";
inline constexpr const char* BRIDGE_NODE    = "r3d_ws_bridge";

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
