#pragma once
// POSIX termios serial port — C++ replacement for pyserial.
//
// Ported from test_commu/serial_port.{h,cpp}: configurable frame format
// (data bits / parity / stop bits) and original-termios restoration on close.
// Read model kept non-blocking (O_NONBLOCK + available()/read()) so the recv
// loop in serial_protocol.cpp can poll with FIONREAD.

#include <cstdint>
#include <string>
#include <sys/types.h>
#include <termios.h>

namespace r3d::serial {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Open the device with the given frame format. Returns 0 on success, -1 on
    // failure. Defaults are 8N1.
    int open(const std::string& device, int baud,
             int dataBits = 8, char parity = 'N', int stopBits = 1);
    void close();

    bool is_open() const { return fd_ >= 0; }

    // Write raw bytes. Returns bytes written or <0 on error.
    ssize_t write(const unsigned char* data, std::size_t len);
    ssize_t write(const std::string& data);

    // Bytes available to read without blocking (FIONREAD). Returns 0 if none
    // or on error.
    int available() const;

    // Non-blocking read of up to max pending bytes. Returns bytes read (0 if
    // none), or <0 on error.
    ssize_t read(unsigned char* buf, std::size_t max);

private:
    int fd_ = -1;
    struct termios oldtio_{};
    bool restore_ = false;   // oldtio_ valid → restore on close

    int setup(int baud, int dataBits, char parity, int stopBits);
};

// Translate a baud rate int to the termios speed_t constant.
int baud_to_constant(int baud);

} // namespace r3d::serial
