#pragma once
// POSIX termios serial port — C++ replacement for pyserial.

#include <string>

namespace r3d::serial {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Open the port at the given baud (8N1, non-blocking reads with a short
    // VMIN/VTIME so the rx loop can poll). Returns false on failure.
    bool open(const std::string& port, int baud);
    void close();

    bool is_open() const { return fd_ >= 0; }

    // Write raw bytes. Returns bytes written or <0 on error.
    int write(const std::string& data);
    int write(const unsigned char* data, std::size_t len);

    // Number of bytes available to read without blocking (0 if none / error).
    int available() const;

    // Read up to max bytes that are currently pending. Returns bytes read.
    int read(unsigned char* buf, std::size_t max);

    void flush_input();

private:
    int fd_ = -1;
};

// Translate a baud rate int to the termios speed_t constant.
int baud_to_constant(int baud);

} // namespace r3d::serial
