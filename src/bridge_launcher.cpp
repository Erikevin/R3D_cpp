// BridgeLauncher implementation — fork + gnome-terminal, mirrors daemon.cpp's
// spawn_main() pattern and Auto_start.py's launch command.

#include "r3d/bridge_launcher.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>

#include "r3d/config.hpp"
#include "r3d/logger.hpp"

namespace r3d {

namespace {
std::string ros_ws() {
    if (const char* env = std::getenv(config::ROS_WS_ENV)) return env;
    return config::ROS_WS_DEFAULT;
}
} // namespace

BridgeLauncher::~BridgeLauncher() { stop(); }

void BridgeLauncher::start() {
    if (pid_ > 0) return;  // already launched
    // Mirror Auto_start.py: gnome-terminal -- bash -c 'source ROS + ws && ros2 run ...; exec bash'
    const std::string cmd =
        std::string("source ") + config::ROS_SETUP +
        " && source " + ros_ws() + "/install/setup.bash" +
        " && ros2 run " + config::BRIDGE_PACKAGE + " " + config::BRIDGE_NODE +
        "; exec bash";

    log::info("[bridge] 启动 r3d_bringup bridge（gnome-terminal）...");
    pid_t pid = fork();
    if (pid < 0) {
        log::warn("[bridge] fork 失败: " + std::string(std::strerror(errno)));
        return;
    }
    if (pid == 0) {
        execlp(config::TERMINAL_CMD, config::TERMINAL_CMD, "--",
               "bash", "-c", cmd.c_str(), (char*)nullptr);
        std::perror("[bridge] execlp gnome-terminal");
        _exit(127);
    }
    pid_ = pid;
    log::info(std::string("[bridge] gnome-terminal 已启动 pid=") + std::to_string(pid));
}

void BridgeLauncher::stop() {
    if (pid_ > 0) {
        kill(pid_, SIGTERM);
        // gnome-terminal returns immediately (server mode); reap without blocking.
        int status = 0;
        for (int i = 0; i < 10; ++i) {
            if (waitpid(pid_, &status, WNOHANG) != 0) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        pid_ = -1;
    }
    // The ros2 child is reparented (gnome-terminal forks it off); pkill by cmd.
    const std::string pkill = std::string("pkill -f \"ros2 run ") +
                              config::BRIDGE_PACKAGE + " " + config::BRIDGE_NODE + "\"";
    int rc = std::system(pkill.c_str());
    (void)rc;
    log::info("[bridge] 已请求停止 r3d_bringup bridge");
}

} // namespace r3d
