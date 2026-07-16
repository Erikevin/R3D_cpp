#pragma once
// Power-board UART protocol — C++ mirror of R3D/serial_protocol.py.
//
// recv loop scans incoming bytes for the shutdown frame (5C 31 .. 01 0D 0A);
// on detection it sends ALLOW_SHUTDOWN and invokes the shutdown callback.

#include <atomic>
#include <functional>
#include <string>
#include <thread>

#include "serial.hpp"

namespace r3d {

class SerialProtocol {
public:
    // Called when the board requests a shutdown. Wired to System::shutdown().
    using ShutdownCallback = std::function<void()>;

    SerialProtocol() = default;
    ~SerialProtocol();

    SerialProtocol(const SerialProtocol&) = delete;
    SerialProtocol& operator=(const SerialProtocol&) = delete;

    void set_shutdown_callback(ShutdownCallback cb) { on_shutdown_ = std::move(cb); }

    bool open();              // opens config port + starts recv thread
    void close();             // stops recv thread + closes port
    bool send(const std::string& data);

private:
    void recv_task();         // background rx loop

    serial::SerialPort ser_;
    std::atomic<bool> running_{false};
    std::thread recv_thread_;
    std::thread::id  recv_tid_;     // so close() can avoid joining itself
    ShutdownCallback on_shutdown_;
};

} // namespace r3d
