// r3d_main — entry point for the Tier-A core control process.
// Starts the cross-process command monitor (mirrors main.py::monitor_shared_cmd)
// and runs the System state machine + debug console.

#include <atomic>
#include <chrono>
#include <csignal>
#include <stdexcept>
#include <thread>

#include "r3d/config.hpp"
#include "r3d/ipc.hpp"
#include "r3d/logger.hpp"
#include "r3d/system.hpp"

namespace r3d {
// Definition of the run flag declared in system.hpp.
std::atomic<bool> g_running{true};
}

static void install_signals() {
    auto handler = +[](int) { r3d::g_running = false; };
    std::signal(SIGINT, handler);
    std::signal(SIGTERM, handler);
}

// Poll /tmp/shared_cmd.txt: 1 = open radar / start mapping, 0 = close radar.
static void monitor_shared_cmd(r3d::System& system) {
    r3d::log::info("[系统] 跨进程指令监听已启动");
    r3d::log::info("[规则] 1=开雷达 | 0=关雷达");

    int last_cmd = -1;   // only act when the command changes
    while (r3d::g_running) {
        int cmd = -1;
        if (r3d::ipc::read_cmd(r3d::config::SHARE_FILE, cmd)) {
            if (cmd != last_cmd &&
                (cmd == r3d::config::CMD_RADAR_OFF || cmd == r3d::config::CMD_RADAR_ON)) {
                last_cmd = cmd;
                try {
                    if (cmd == r3d::config::CMD_RADAR_ON) {
                        r3d::log::info("【跨进程指令】→ 开启雷达");
                        system.start_radar();
                    } else {
                        r3d::log::info("【跨进程指令】→ 关闭雷达");
                        system.stop_radar();
                    }
                } catch (const std::exception& e) {
                    r3d::log::error(std::string("雷达指令异常: ") + e.what());
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(r3d::config::IPC_POLL_MS));
    }
}

int main() {
    install_signals();
    r3d::System system;

    std::thread ipc_thread([&system] { monitor_shared_cmd(system); });

    try {
        system.start_main_flow();
    } catch (const std::exception& e) {
        r3d::log::error(std::string("异常：") + e.what());
    }

    // finally-style cleanup (mirrors the Python finally block): stop the monitor,
    // turn LEDs off, close the UART.
    r3d::g_running = false;
    system.hw().exit_clean();
    system.serial().close();
    if (ipc_thread.joinable()) ipc_thread.join();
    return 0;
}
