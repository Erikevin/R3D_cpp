// BridgeLauncher implementation — fork + gnome-terminal, mirrors daemon.cpp's
// spawn_main() pattern and Auto_start.py's launch command.

#include "r3d/bridge_launcher.hpp"

#include <fcntl.h>      // open() for headless log redirection
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
    std::lock_guard<std::mutex> lk(m_);
    if (pid_ > 0) {                      // reap any zombie, then no-op if still alive
        pid_t r = ::waitpid(pid_, nullptr, WNOHANG);
        if (r == 0) return;              // still running
        pid_ = -1;                       // it had died
    }

    // Free port 8765 before (re)binding: kill whatever holds it. The operator has
    // confirmed nothing important uses 8765, so this is safe — it clears a stale
    // orphaned bridge from a previous crash as well as any other accidental holder.
    int kill_rc = std::system("fuser -k 8765/tcp 2>/dev/null; pkill -f r3d_ws_bridge 2>/dev/null");
    (void)kill_rc;
    // Fail fast if the workspace can't be sourced. A wrong path here used to
    // silently abort the command chain inside the child (source failed → ros2
    // run never ran) while the parent still logged "started". Override via
    // $R3D_ROS_WS (dev box vs Jetson deploy path).
    const std::string setup_bash = ros_ws() + "/install/setup.bash";
    if (access(setup_bash.c_str(), R_OK) != 0) {
        log::error("[bridge] workspace setup 不可读: " + setup_bash +
                   "（用 " + std::string(config::ROS_WS_ENV) +
                   "=<ros2_ws 路径> 覆盖默认值），放弃启动");
        return;
    }
    // Source ROS + the workspace, then `exec ros2 run ...` so the node replaces
    // bash and pid_ tracks the launcher process itself. (The trailing
    // `; exec bash` is added only for the interactive terminal-window path.)
    const std::string ros_cmd =
        std::string("source ") + config::ROS_SETUP +
        " && source " + ros_ws() + "/install/setup.bash" +
        " && exec ros2 run " + config::BRIDGE_PACKAGE + " " + config::BRIDGE_NODE;

    // Prefer an interactive gnome-terminal window on dev desktops; fall back to a
    // detached, logged bash session on headless hosts (no gnome-terminal / no
    // DISPLAY) — which is exactly where the bridge must run on the real device.
    const bool have_display = std::getenv("DISPLAY") || std::getenv("WAYLAND_DISPLAY");
    const bool have_terminal = have_display && access(config::TERMINAL_CMD, X_OK) == 0;

    log::info(std::string("[bridge] 启动 r3d_bringup bridge（") +
              (have_terminal ? "gnome-terminal" : "headless") + ")...");
    pid_t pid = fork();
    if (pid < 0) {
        log::warn("[bridge] fork 失败: " + std::string(std::strerror(errno)));
        return;
    }
    if (pid == 0) {
        setsid();  // own session: survives r3d_main, needs no controlling tty
        // Both branches redirect the launcher process's own stdout/stderr to
        // BRIDGE_LOG. On success this stays near-empty; on failure (e.g. SSH
        // with no X authority / D-Bus, where gnome-terminal can't open a window)
        // the real error lands here instead of being lost.
        int fd = open(config::BRIDGE_LOG, O_WRONLY | O_CREAT | O_APPEND, 0644);
        if (fd >= 0) {
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            close(fd);
        }
        if (have_terminal) {
            // keep the window open after exit for debugging
            const std::string cmd = ros_cmd + "; exec bash";
            execlp(config::TERMINAL_CMD, config::TERMINAL_CMD, "--",
                   "bash", "-c", cmd.c_str(), (char*)nullptr);
        } else {
            execlp("bash", "bash", "-c", ros_cmd.c_str(), (char*)nullptr);
        }
        std::perror("[bridge] execlp");
        _exit(127);
    }
    pid_ = pid;

    // gnome-terminal runs in server mode: the launcher process exits quickly
    // (code 0) once it has handed off to the server, so reaping a fast exit is
    // normal and does NOT mean the node failed. A non-zero exit means the
    // launch itself failed (SSH / no graphics → window never opened → ros2 run
    // never ran). In the headless branch pid_ is the node itself, which stays
    // alive and is therefore NOT reaped here.
    int status = 0;
    bool reaped = false;
    for (int i = 0; i < 20; ++i) {  // poll up to ~1s; return as soon as it exits
        if (waitpid(pid_, &status, WNOHANG) != 0) { reaped = true; break; }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (reaped) {
        int code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
        pid_ = -1;  // launcher reaped; the real node (if any) is cleaned via stop()'s pkill
        if (code == 0) {
            log::info("[bridge] bridge 启动器已正常移交（gnome-terminal server 模式）；"
                      "完整日志见 " + std::string(config::BRIDGE_LOG));
        } else {
            log::error("[bridge] bridge 启动失败（退出码=" + std::to_string(code) +
                       "）。SSH/无图形会话下常因 gnome-terminal 缺 X 授权或 D-Bus 打不开窗口；"
                       "真实报错见 " + std::string(config::BRIDGE_LOG));
        }
    } else {
        log::info(std::string("[bridge] bridge 已派生 pid=") +
                  std::to_string(pid_) + " 并持续运行；若桌面未见窗口，请查 " +
                  std::string(config::BRIDGE_LOG));
    }
}

void BridgeLauncher::stop() {
    std::lock_guard<std::mutex> lk(m_);
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
    // Match by node name: the child runs `exec ros2 run …`, which replaces the
    // launcher image with the r3d_ws_bridge node, so the final process cmdline is
    // r3d_ws_bridge — the old "ros2 run …" pattern never matched it.
    const std::string pkill = std::string("pkill -f ") + config::BRIDGE_NODE;
    int rc = std::system(pkill.c_str());
    (void)rc;
    log::info("[bridge] 已请求停止 r3d_bringup bridge");
}

bool BridgeLauncher::is_alive() {
    std::lock_guard<std::mutex> lk(m_);
    if (pid_ > 0) {
        pid_t r = ::waitpid(pid_, nullptr, WNOHANG);
        if (r == 0) return true;   // tracked child still running
        pid_ = -1;                 // reaped (r == pid) or already gone (ECHILD) → cleared
        // fall through: the child we tracked is gone, but in gnome-terminal mode
        // that child was just the launcher, which exits 0 on hand-off — judge by
        // the node's actual presence instead.
    }
    // pid_ <= 0: never forked, or the launcher was reaped after handing off to the
    // terminal server. Either way pid_ can't track the real node long-term, so match
    // by node name — consistent with stop()'s `pkill -f r3d_ws_bridge`. Headless mode
    // exec-replaces the launcher into r3d_ws_bridge, so the name matches there too.
    std::string cmd = std::string("/usr/bin/pgrep -f ") + config::BRIDGE_NODE +
                      " >/dev/null 2>&1";
    int rc = std::system(cmd.c_str());
    return WIFEXITED(rc) && WEXITSTATUS(rc) == 0;
}

} // namespace r3d
