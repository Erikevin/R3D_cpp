// Battery / fault protection — passive monitor (mirrors protection.py).

#include "r3d/protection.hpp"

#include <chrono>
#include <thread>

#include "r3d/config.hpp"
#include "r3d/system.hpp"

namespace r3d {

Protection::Protection(System& sys) : system_(sys) {}

Protection::~Protection() { stop(); }

void Protection::start_monitor() {
    running_ = true;
    if (thread_.joinable()) return;
    thread_ = std::thread([this] { monitor_task(); });
}

void Protection::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void Protection::monitor_task() {
    // The Python monitor simply sleeps; battery frames are not yet parsed.
    // Kept as a 1 s heartbeat ready for future low-battery handling.
    while (running_) {
        std::this_thread::sleep_for(std::chrono::seconds(config::PROTECTION_HEARTBEAT_SEC));
    }
}

} // namespace r3d
