// Power-board protocol implementation.

#include "r3d/serial_protocol.hpp"

#include <chrono>
#include <cstring>
#include <thread>
#include <vector>

#include "r3d/config.hpp"
#include "r3d/logger.hpp"

namespace r3d {

SerialProtocol::~SerialProtocol() { close(); }

bool SerialProtocol::open() {
    if (!ser_.open(config::UART_PORT, config::UART_BAUD)) {
        log::error("[串口] 打开失败: " + config::UART_PORT);
        return false;
    }
    running_ = true;
    recv_thread_ = std::thread([this] { recv_task(); });
    recv_tid_ = recv_thread_.get_id();
    log::info("[INFO] 串口打开成功 -> " + config::UART_PORT +
              " " + std::to_string(config::UART_BAUD));
    return true;
}

void SerialProtocol::close() {
    running_ = false;
    // Never join from within the recv thread itself: the shutdown callback runs
    // on the recv thread and calls System::shutdown() -> close(). Joining ourself
    // would deadlock, so skip the join in that case (the loop exits on running_).
    if (recv_thread_.joinable() && std::this_thread::get_id() != recv_tid_) {
        recv_thread_.join();
    }
    ser_.close();
}

bool SerialProtocol::send(const std::string& data) {
    if (!ser_.is_open()) return false;
    return ser_.write(data) == static_cast<int>(data.size());
}

void SerialProtocol::recv_task() {
    // Header 5C 31 = shutdown cmd; the payload byte at the configured offset
    // decides the action (mirrors the Python byte layout).
    const unsigned char SHUTDOWN_HDR[] = {config::FRAME_HEADER, config::SHUTDOWN_CMD_BYTE};
    std::vector<unsigned char> buf;

    while (running_) {
        int n = ser_.available();
        if (n > 0) {
            buf.resize(static_cast<std::size_t>(n));
            int got = ser_.read(buf.data(), buf.size());
            if (got > 0) {
                buf.resize(static_cast<std::size_t>(got));

                // Search for the header, then check the payload byte at the offset.
                for (std::size_t i = 0;
                     i + config::SHUTDOWN_PAYLOAD_OFFSET < buf.size(); ++i) {
                    if (buf[i] == SHUTDOWN_HDR[0] && buf[i + 1] == SHUTDOWN_HDR[1]) {
                        if (buf[i + config::SHUTDOWN_PAYLOAD_OFFSET] ==
                            config::SHUTDOWN_PAYLOAD_VALUE) {
                            log::info("[串口] 收到关机指令 (5C 31 .. 01 0D 0A)");
                            log::info("[串口] 发送：允许关机");
                            send(config::ALLOW_SHUTDOWN);
                            std::this_thread::sleep_for(std::chrono::milliseconds(config::ALLOW_SHUTDOWN_DELAY_MS));
                            if (on_shutdown_) on_shutdown_();
                        }
                    }
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(config::SERIAL_RECV_POLL_MS));
    }
}

} // namespace r3d
