#pragma once
// Central state machine + orchestration — C++ mirror of R3D/system.py.

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "bridge_launcher.hpp"
#include "config.hpp"
#include "hardware.hpp"
#include "protection.hpp"
#include "serial_protocol.hpp"

namespace r3d {

// Global run flag — set false by SIGINT/SIGTERM (defined in main.cpp).
// The state-machine loop and background threads poll this to shut down cleanly.
extern std::atomic<bool> g_running;

class System {
public:
    System();
    ~System();

    System(const System&) = delete;
    System& operator=(const System&) = delete;

    // ---- state ----
    void set_state(config::State s);
    config::State state() const { return state_; }
    void update_led();

    // ---- checks ----
    bool check_camera() const;          // mirrors config.DEBUG_SKIP_CAMERA path

    // ---- lifecycle ----
    void boot_wait();
    bool init_all();                    // hardware + serial
    // True once init_all() has succeeded — gates cross-process command handling
    // so the radar is never commanded before GPIO/serial are ready.
    bool initialized() const { return initialized_.load(); }
    void start_radar();
    void stop_radar();
    void shutdown();                    // clean shutdown sequence

    // ---- main flow ----
    // Blocks (debug console / state loop). Returns the process exit code: 0 on a
    // clean stop, non-zero if the bridge died (so r3d-main.service restarts us).
    int start_main_flow();

    // Expose subsystems (Protection needs a back-reference).
    Hardware&       hw()       { return hw_; }
    SerialProtocol& serial()   { return serial_; }

private:
    void debug_console();
    // Block (up to timeout_sec, 500 ms steps) until the mapping children
    // (livox/lio_sam/image_odom_recorder/bag) are gone; SIGKILL stragglers on timeout.
    void wait_mapping_idle(int timeout_sec);
    // Background supervisor: if the bridge dies, set bridge_died_ and stop the main
    // loop so r3d_main exits non-zero → r3d-main.service restarts (which first kills
    // whoever holds 8765, then relaunches the bridge).
    void monitor_bridge();

    Hardware        hw_;
    SerialProtocol  serial_;
    Protection      protect_;
    BridgeLauncher  bridge_;
    config::State   state_ = config::State::Init;
    std::atomic<bool> debug_mode_{false};
    std::atomic<bool> shutting_down_{false};
    std::atomic<bool> initialized_{false};  // set true at end of init_all()

    std::atomic<bool> bridge_died_{false};   // set by monitor_bridge() on bridge exit
    std::atomic<bool> bridge_confirmed_alive_{false};  // set once monitor sees the bridge alive
    std::thread bridge_mon_;                  // runs monitor_bridge()
};

} // namespace r3d
