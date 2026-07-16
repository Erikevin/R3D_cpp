#pragma once
// Central state machine + orchestration — C++ mirror of R3D/system.py.

#include <atomic>
#include <memory>

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
    void start_radar();
    void stop_radar();
    void shutdown();                    // clean shutdown sequence

    // ---- main flow ----
    void start_main_flow();             // blocks (debug console / state loop)

    // Expose subsystems (Protection needs a back-reference).
    Hardware&       hw()       { return hw_; }
    SerialProtocol& serial()   { return serial_; }

private:
    void debug_console();

    Hardware        hw_;
    SerialProtocol  serial_;
    Protection      protect_;
    BridgeLauncher  bridge_;
    config::State   state_ = config::State::Init;
    std::atomic<bool> debug_mode_{false};
    std::atomic<bool> shutting_down_{false};
};

} // namespace r3d
