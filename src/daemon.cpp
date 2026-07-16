// r3d_daemon — watchdog that respawns r3d_main in a new terminal window.
// Mirrors R3D/daemon.py: clean exit (rc 0) stops the daemon; a crash restarts it.

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/wait.h>

#include "r3d/config.hpp"

namespace {
volatile sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }

// Path to the main binary (overridable via $R3D_MAIN_BIN for testing/deploy).
const char* main_bin() {
    if (const char* env = std::getenv(r3d::config::MAIN_BIN_ENV)) return env;
    return r3d::config::MAIN_BIN_DEFAULT;
}

// Launch r3d_main inside a new gnome-terminal; block until it exits.
// Returns the child's exit code, or -1 on launch failure.
int spawn_main() {
    pid_t pid = fork();
    if (pid < 0) return -1;
    if (pid == 0) {
        execlp(r3d::config::TERMINAL_CMD, r3d::config::TERMINAL_CMD, "--", main_bin(), nullptr);
        std::perror("execlp gnome-terminal");
        _exit(127);
    }
    int status = 0;
    while (waitpid(pid, &status, 0) < 0) {
        if (g_stop) return -1;
    }
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    return -1;  // killed by signal etc. -> treat as crash
}
} // namespace

int main() {
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::cout << "==== 守护进程已启动 ====\n"
                 "新窗口启动 " << main_bin() << "\n"
                 "Ctrl+C = 双进程关闭 | 异常崩溃 = 自动重启\n";

    while (!g_stop) {
        std::cout << "启动主进程（新终端窗口）\n";
        int rc = spawn_main();
        if (g_stop) break;

        if (rc == 0) {
            std::cout << "主进程正常关闭，守护进程同步退出\n";
            break;
        }
        std::cout << "主进程异常，" << r3d::config::DAEMON_RESTART_SEC << "秒后重启...\n";
        for (int i = 0; i < r3d::config::DAEMON_RESTART_SEC * 10 && !g_stop; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
