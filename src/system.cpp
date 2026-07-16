// Central state machine + orchestration — implementation of system.hpp.
// Mirrors R3D/system.py.

#include "r3d/system.hpp"

#include <cctype>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>   // access(), execlp()

#include "r3d/config.hpp"
#include "r3d/logger.hpp"

namespace r3d {

// Normalize a console line: trim whitespace + lowercase (Python .strip().lower()).
static std::string normalize(std::string s) {
    auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    auto b = s.find_last_not_of(" \t\r\n");
    s = s.substr(a, b - a + 1);
    for (auto& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

System::System() : protect_{*this} {
    // Wire the power-board shutdown request into our clean shutdown sequence
    // (the Python original did `self.serial.system = self`).
    serial_.set_shutdown_callback([this] { shutdown(); });
}

System::~System() {
    hw_.exit_clean();
    serial_.close();
}

void System::set_state(config::State s) {
    state_ = s;
    update_led();
}

void System::update_led() {
    hw_.green(false);
    hw_.red(false);
    switch (state_) {
        case config::State::Run:        hw_.green(true); break;
        case config::State::Error:      hw_.red(true);   break;
        case config::State::LowBattery: hw_.red(true);   break;
        default: break;
    }
}

bool System::check_camera() const {
    if (config::DEBUG_SKIP_CAMERA) return true;
    return ::access(config::VIDEO_DEV_0, F_OK) == 0 &&
           ::access(config::VIDEO_DEV_2, F_OK) == 0;
}

void System::boot_wait() {
    std::this_thread::sleep_for(std::chrono::seconds(config::BOOT_WAIT_SEC));
}

bool System::init_all() {
    if (!hw_.init()) return false;
    if (!serial_.open()) return false;
    return true;
}

void System::start_radar() {
    if (!check_camera()) {
        log::warn("[摄像头离线] 不执行");
        return;
    }
    log::info("[执行] 雷达电源打开 → 绿灯闪烁");
    set_state(config::State::Scan);
    hw_.start_scan_pwm();
    serial_.send(config::RADAR_ON);
    std::this_thread::sleep_for(std::chrono::seconds(config::RADAR_RESEND_SEC));
    serial_.send(config::RADAR_ON);   // sent twice, as in Python
}

void System::stop_radar() {
    if (!check_camera()) {
        log::warn("[摄像头离线] 不执行");
        return;
    }
    log::info("[执行] 雷达电源关闭 → 绿灯常亮");
    hw_.stop_scan_pwm();
    set_state(config::State::Run);
    serial_.send(config::RADAR_OFF);
    std::this_thread::sleep_for(std::chrono::seconds(config::RADAR_RESEND_SEC));
    serial_.send(config::RADAR_OFF);
}

// ============================== 系统关机（干净、安全） ==============================
void System::shutdown() {
    // The recv thread and the debug console can both reach here; guard re-entry.
    if (shutting_down_.exchange(true)) return;

    log::info("[系统] 开始关机...");
    set_state(config::State::Shutdown);
    hw_.exit_clean();
    serial_.close();

    // Python used a broken `echo 123 | sudo -S shutdown` pipe. We exec the real
    // shutdown; on success the process image is replaced and we never return.
    execlp(config::SHUTDOWN_BIN, config::SHUTDOWN_BIN, "-h", "now", nullptr);

    // Reached only if exec failed (binary missing / not root). Stay tidy and exit.
    log::error("[系统] shutdown 执行失败 — 请以 root 运行或确认 shutdown 已安装");
    g_running = false;
}

void System::debug_console() {
    std::cout << "\n==== 调试模式已开启 ===\n"
                 "指令：\n"
                 "  start    → 雷达开启（闪烁+发指令）\n"
                 "  stop     → 雷达关闭（常亮+发指令）\n"
                 "  shutdown → 手动关机\n"
                 "  exit     → 退出调试\n";

    std::string line;
    while (g_running) {
        std::cout << "输入指令：" << std::flush;
        if (!std::getline(std::cin, line)) {
            // EOF (Ctrl+D) / closed stdin. Python raises EOFError here and the
            // program exits — replicate that instead of looping on the prompt.
            g_running = false;
            break;
        }
        const std::string cmd = normalize(line);

        if (cmd == "start") {
            start_radar();
        } else if (cmd == "stop") {
            stop_radar();
        } else if (cmd == "shutdown") {
            shutdown();
            break;
        } else if (cmd == "exit") {
            std::cout << "退出调试模式\n";
            debug_mode_ = false;
            break;
        } else if (!cmd.empty()) {
            std::cout << "未知指令\n";
        }
    }
}

void System::start_main_flow() {
    log::info("==== 系统启动 ====");
    boot_wait();

    // Mirror Auto_start.py: bring up the bridge (r3d_bringup) early so the mobile
    // client can connect as soon as the device boots.
    bridge_.start();

    if (!init_all()) {
        log::error("初始化失败");
        set_state(config::State::Error);
    } else {
        log::info("初始化成功");
        set_state(config::State::Run);
    }

    protect_.start_monitor();

    while (g_running) {
        if (!check_camera()) {
            set_state(config::State::Error);
            std::this_thread::sleep_for(std::chrono::milliseconds(config::CAMERA_ERROR_RECHECK_MS));
            continue;
        }
        if (state_ == config::State::Error) {
            set_state(config::State::Run);
        }
        if (!debug_mode_) {
            std::cout << "\n输入 'debug' 开启调试模式" << std::endl;
            debug_mode_ = true;
            debug_console();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(config::MAIN_LOOP_IDLE_MS));
    }
}

} // namespace r3d
