// POSIX termios serial implementation.

#include "r3d/serial.hpp"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <cstring>

#include "r3d/config.hpp"
#include "r3d/logger.hpp"

namespace r3d::serial {

int baud_to_constant(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        default:     return B9600;
    }
}

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(const std::string& port, int baud) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        log::error("serial: cannot open " + port + " — " + std::strerror(errno));
        return false;
    }

    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        log::error("serial: tcgetattr failed");
        close();
        return false;
    }

    cfsetispeed(&tty, baud_to_constant(baud));
    cfsetospeed(&tty, baud_to_constant(baud));

    // 8N1, no flow control, raw
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | INLCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    // Non-blocking-ish poll: VMIN=0, VTIME=1 (0.1s) so the rx loop can spin.
    tty.c_cc[VMIN]  = config::UART_VMIN;
    tty.c_cc[VTIME] = config::UART_VTIME;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        log::error("serial: tcsetattr failed");
        close();
        return false;
    }
    flush_input();
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

void SerialPort::flush_input() { if (fd_ >= 0) tcflush(fd_, TCIFLUSH); }

int SerialPort::write(const std::string& data) {
    return write(reinterpret_cast<const unsigned char*>(data.data()), data.size());
}

int SerialPort::write(const unsigned char* data, std::size_t len) {
    if (fd_ < 0) return -1;
    return static_cast<int>(::write(fd_, data, len));
}

int SerialPort::available() const {
    if (fd_ < 0) return 0;
    int n = 0;
    if (::ioctl(fd_, FIONREAD, &n) < 0) return 0;
    return n;
}

int SerialPort::read(unsigned char* buf, std::size_t max) {
    if (fd_ < 0) return 0;
    return static_cast<int>(::read(fd_, buf, max));
}

} // namespace r3d::serial
