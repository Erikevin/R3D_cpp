// Power-board protocol implementation.

#include "r3d/serial_protocol.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <vector>

#include "r3d/config.hpp"
#include "r3d/logger.hpp"

namespace r3d {

// Format a byte range as uppercase hex ("5C 33 01 ...") for serial tracing.
static std::string to_hex(const unsigned char* p, std::size_t n) {
    static const char* kHex = "0123456789ABCDEF";
    std::string s;
    s.reserve(n * 3);
    for (std::size_t i = 0; i < n; ++i) {
        if (i) s += ' ';
        s += kHex[p[i] >> 4];
        s += kHex[p[i] & 0x0F];
    }
    return s;
}

SerialProtocol::~SerialProtocol() { close(); }

bool SerialProtocol::open() {
    if (ser_.open(config::UART_PORT, config::UART_BAUD) != 0) {
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
    bool ok = ser_.write(data) == static_cast<ssize_t>(data.size());
    if (ok) {
        log::info("[串口] → 发送: " +
                  to_hex(reinterpret_cast<const unsigned char*>(data.data()), data.size()));
    }
    return ok;
}

void SerialProtocol::recv_task() {
    std::vector<unsigned char> buf;

    while (running_) {
        int n = ser_.available();
        if (n > 0) {
            buf.resize(static_cast<std::size_t>(n));
            int got = ser_.read(buf.data(), buf.size());
            if (got > 0) {
                // Always show the raw received bytes first, then decode frames.
                log::info("[串口] ← 收到(" + std::to_string(got) + "B): " +
                          to_hex(buf.data(), static_cast<std::size_t>(got)));
                parse_buffer(buf.data(), static_cast<std::size_t>(got));
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(config::SERIAL_RECV_POLL_MS));
    }
}

void SerialProtocol::parse_buffer(const unsigned char* data, std::size_t len) {
    // Walk the chunk looking for complete 9-byte 0x31 telemetry frames:
    //   [5C][31][U_H][U_L][Temp][Power_off][Battery][0D][0A]
    // Per-chunk scan (no cross-chunk reassembly); the report is cyclic, so a
    // frame split across two reads is simply caught on the next cycle.
    if (len < static_cast<std::size_t>(config::FRAME_LEN)) return;

    for (std::size_t i = 0; i + config::FRAME_LEN <= len; ++i) {
        if (data[i] != config::FRAME_HEADER) continue;
        if (data[i + 1] != config::TELEMETRY_CMD) continue;
        // Validate the 0x0D 0x0A tail before trusting the payload.
        if (data[i + 7] != static_cast<unsigned char>(config::FRAME_END[0]) ||
            data[i + 8] != static_cast<unsigned char>(config::FRAME_END[1])) continue;

        unsigned volt_h    = data[i + config::TELEMETRY_VOLT_H_OFF];
        unsigned volt_l    = data[i + config::TELEMETRY_VOLT_L_OFF];
        unsigned temp      = data[i + config::TELEMETRY_TEMP_OFF];
        unsigned power_off = data[i + config::TELEMETRY_POWER_OFF_OFF];
        unsigned battery   = data[i + config::TELEMETRY_BATT_OFF];
        unsigned voltage_mv = (volt_h << 8) | volt_l;   // millivolts

        // Decoded telemetry — the meaningful print of the received 0x31 frame.
        char vbuf[16];
        std::snprintf(vbuf, sizeof(vbuf), "%.2f", voltage_mv / 1000.0);
        log::info("[串口] 遥测: 电压=" + std::string(vbuf) + "V" +
                  "  温度=" + std::to_string(temp) +
                  "  电量=" + std::to_string(battery) + "%" +
                  "  关机请求=" + (power_off > 0 ? "是" : "否"));

        // Auto fan (thermal): edge-triggered on the temperature threshold — one
        // FAN_ON when crossing above, one FAN_OFF when crossing back, so the
        // cyclic report never spams fan commands.
        if (temp > static_cast<unsigned>(config::FAN_TEMP_THRESHOLD)) {
            if (!fan_auto_on_) {
                fan_auto_on_ = true;
                log::info("[串口] 温度=" + std::to_string(temp) + "°C > " +
                          std::to_string(config::FAN_TEMP_THRESHOLD) + "°C，自动开启风扇");
                send(config::FAN_ON);
            }
        } else if (fan_auto_on_) {
            fan_auto_on_ = false;
            log::info("[串口] 温度=" + std::to_string(temp) + "°C <= " +
                      std::to_string(config::FAN_TEMP_THRESHOLD) + "°C，自动关闭风扇");
            send(config::FAN_OFF);
        }

        // Shutdown triggers (spec 5.2), checked on every frame (not throttled):
        //   - Power_off > 0       : board explicitly requests shutdown
        //   - battery < BAT_LOW   : low battery (<10%)
        // Either enters the shutdown flow and sends ALLOW_SHUTDOWN. Latched so
        // the cyclic report does not re-trigger it on every frame.
        if (!shutdown_triggered_ &&
            (power_off > 0 || battery < config::BAT_LOW)) {
            shutdown_triggered_ = true;
            const char* reason = (power_off > 0) ? "Power_off>0" : "电量不足(<10%)";
            log::info(std::string("[串口] 触发关机流程 (") + reason + ")");
            log::info("[串口] 发送：允许关机");
            send(config::ALLOW_SHUTDOWN);
            std::this_thread::sleep_for(std::chrono::milliseconds(config::ALLOW_SHUTDOWN_DELAY_MS));
            if (on_shutdown_) on_shutdown_();
        }
    }
}

} // namespace r3d
