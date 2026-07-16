#pragma once
// BridgeLauncher — spawns the r3d_bringup WebSocket bridge in a gnome-terminal,
// mirroring R3D/Auto_start.py's web_a.py launch. Keeps r3d_main (Tier-A) free of
// any ROS link dependency: the ROS environment is sourced inside the terminal.

#include <sys/types.h>

namespace r3d {

class BridgeLauncher {
public:
    BridgeLauncher() = default;
    ~BridgeLauncher();

    BridgeLauncher(const BridgeLauncher&) = delete;
    BridgeLauncher& operator=(const BridgeLauncher&) = delete;

    // Open a gnome-terminal that sources ROS + the workspace and runs
    // `ros2 run r3d_bringup r3d_ws_bridge`. No-op if already running; logs and
    // returns on failure (e.g. headless host with no DISPLAY) — never throws.
    void start();

    // Terminate the spawned bridge process group (best-effort).
    void stop();

private:
    pid_t pid_{-1};
};

} // namespace r3d
