// Central state machine + orchestration — implementation of system.hpp.
// Mirrors R3D/system.py.

#include "r3d/system.hpp"

#include <cctype>
#include <cerrno>
#include <chrono>
#include <iostream>
#include <string>
#include <sys/select.h>
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
    // Stop the bridge supervisor first so it can't touch bridge_ during teardown.
    // g_running is already false (main's cleanup sets it before returning), so the
    // loop falls out within its 100 ms slice.
    if (bridge_mon_.joinable()) bridge_mon_.join();
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
    initialized_ = true;   // unblocks monitor_shared_cmd() to act on commands
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
    const int in_fd = STDIN_FILENO;
    while (g_running) {
        std::cout << "输入指令：" << std::flush;
        // 用 select 带超时轮询 stdin，而非阻塞在 std::getline：libstdc++ filebuf
        // 对底层 read 的 EINTR 会自动重试，所以即便信号 handler 置了 g_running，
        // getline 也不会因 Ctrl+C 返回 → r3d_main 卡死、析构链(停桥/清建图子进程)
        // 不触发。改为每 0.5s 复查 g_running：Ctrl+C 后 handler 置 false，select
        // 超时(或被 EINTR 中断)后即退出循环。serial 是 O_NONBLOCK，不受影响。
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(in_fd, &fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000;  // 0.5s
        int rc = select(in_fd + 1, &fds, nullptr, nullptr, &tv);
        if (rc < 0) {
            if (errno == EINTR && g_running) continue;
            g_running = false;
            break;
        }
        if (rc == 0 || !FD_ISSET(in_fd, &fds)) continue;  // 超时 → 复查 g_running
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

int System::start_main_flow() {
    log::info("==== 系统启动 ====");
    boot_wait();

    // Mirror Auto_start.py: bring up the bridge (r3d_bringup) early so the mobile
    // client can connect as soon as the device boots.
    bridge_.start();
    // Watch the bridge: if it dies (e.g. port 8765 busy → bind failure), tear the
    // whole process down so r3d-main.service restarts us — and on that restart
    // start() first kills whoever holds 8765, then relaunches. Runs off the main
    // thread since debug_console()'s select loop is bypassed while the camera is
    // offline.
    bridge_mon_ = std::thread(&System::monitor_bridge, this);

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
            debug_mode_ = true;
            // 仅在交互式终端（手动 sudo ./r3d_main）开调试控制台。作为 systemd 服务
            // 运行时 stdin=/dev/null，getline 会立即 EOF 并把 g_running 置 false，导致
            // 服务秒退（rc=0，on-failure 不重启）→ 永久 inactive。无 tty 时主循环纯保活
            // 即可——桥监控、IPC 监听各自在独立线程运行。
            if (::isatty(STDIN_FILENO)) {
                std::cout << "\n输入 'debug' 开启调试模式" << std::endl;
                debug_console();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(config::MAIN_LOOP_IDLE_MS));
    }
    // Bridge death sets bridge_died_ and breaks the loop → return non-zero so
    // systemd restarts; a clean SIGINT/SIGTERM stop leaves it 0.
    return bridge_died_ ? 1 : 0;
}

// Watch the bridge; on exit, flag it and unblock the main loop so r3d_main returns
// non-zero and r3d-main.service restarts (freeing 8765, then relaunching the bridge).
void System::monitor_bridge() {
    while (g_running) {
        for (int i = 0; i < 10 && g_running; ++i)   // 1 s heartbeat, 100 ms slices
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!g_running) break;
        if (!bridge_.is_alive()) {
            log::error("[bridge] 桥已退出 —— r3d_main 将随之退出，由 r3d-main.service 重启"
                       "（重启时会先杀掉 8765 占用者再拉起桥）");
            bridge_died_ = true;
            g_running = false;   // unblock the main loop → cleanup → non-zero return
            return;
        }
    }
}

} // namespace r3d
