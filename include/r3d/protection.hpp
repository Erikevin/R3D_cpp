#pragma once
// Battery / fault protection — C++ mirror of R3D/protection.py.
// The Python version is mostly a stub (passive monitor); we preserve that
// behavior and add a clean hook for future battery parsing from the power board.

#include <atomic>
#include <thread>

namespace r3d {

class System; // forward

class Protection {
public:
    explicit Protection(System& sys);
    ~Protection();

    Protection(const Protection&) = delete;
    Protection& operator=(const Protection&) = delete;

    void start_monitor();
    void stop();

    // Hook for future battery-frame parsing (mirrors parse_battery).
    void parse_battery(const unsigned char* /*data*/, std::size_t /*len*/) {}

private:
    void monitor_task();

    System& system_;
    std::atomic<bool> running_{true};
    std::atomic<int>  battery_{100};
    std::thread thread_;
};

} // namespace r3d
