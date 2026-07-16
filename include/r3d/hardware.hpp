#pragma once
// Hardware layer (LEDs + scan blink) — C++ mirror of R3D/hardware.py.
// Owns the GPIO backend and a blink thread that toggles the scan pin @1 Hz.

#include <atomic>
#include <memory>
#include <thread>

#include "gpio.hpp"

namespace r3d {

class Hardware {
public:
    Hardware() = default;
    ~Hardware();

    Hardware(const Hardware&) = delete;
    Hardware& operator=(const Hardware&) = delete;

    // Configure GPIO pins (green/red/scan). Returns false on failure.
    bool init();

    void green(bool on);
    void red(bool on);

    // Start the 1 Hz blink on the scan pin (green LED).
    void start_scan_pwm();
    // Stop blinking; leave the pin HIGH (matches Python).
    void stop_scan_pwm();

    // Turn all outputs off and join the blink thread.
    void exit_clean();

    gpio::IGpio* gpio() { return gpio_.get(); }

private:
    void blink_task();

    std::unique_ptr<gpio::IGpio> gpio_;
    std::atomic<bool> is_scanning_{false};
    std::thread blink_thread_;
};

} // namespace r3d
