// POSIX termios serial implementation.
//
// Frame-format config and termios restoration ported from
// test_commu/serial_port.cpp; non-blocking read model (O_NONBLOCK + FIONREAD)
// kept from the original R3D_cpp version so the recv loop can poll.

#include "r3d/serial.hpp"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>

#include "r3d/logger.hpp"

namespace r3d::serial {

int baud_to_constant(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 921600:  return B921600;
        default:      return -1;
    }
}

SerialPort::~SerialPort() { close(); }

int SerialPort::open(const std::string& device, int baud,
                     int dataBits, char parity, int stopBits) {
    if (is_open()) close();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        log::error("serial: cannot open " + device + " — " + std::strerror(errno));
        return -1;
    }

    // Save the original terminal config so we can restore it on close.
    if (tcgetattr(fd_, &oldtio_) != 0) {
        log::error(std::string("serial: tcgetattr failed — ") + std::strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return -1;
    }
    restore_ = true;

    if (setup(baud, dataBits, parity, stopBits) != 0) {
        ::close(fd_);
        fd_ = -1;
        restore_ = false;
        return -1;
    }

    // Keep O_NONBLOCK: reads return immediately, the recv loop polls with
    // available()/read().
    return 0;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        if (restore_) {
            tcsetattr(fd_, TCSANOW, &oldtio_);  // restore original config
        }
        ::close(fd_);
        fd_ = -1;
        restore_ = false;
    }
}

int SerialPort::setup(int baud, int dataBits, char parity, int stopBits) {
    struct termios tty{};
    std::memset(&tty, 0, sizeof(tty));

    int speed = baud_to_constant(baud);
    if (speed < 0) {
        log::error("serial: unsupported baud rate " + std::to_string(baud));
        return -1;
    }
    cfsetispeed(&tty, static_cast<speed_t>(speed));
    cfsetospeed(&tty, static_cast<speed_t>(speed));

    // Control flags: ignore modem control, enable receiver.
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;

    // Data bits.
    switch (dataBits) {
        case 5: tty.c_cflag |= CS5; break;
        case 6: tty.c_cflag |= CS6; break;
        case 7: tty.c_cflag |= CS7; break;
        case 8: tty.c_cflag |= CS8; break;
        default:
            log::error("serial: invalid data bits " + std::to_string(dataBits));
            return -1;
    }

    // Parity.
    switch (parity) {
        case 'N': case 'n':
            tty.c_cflag &= ~PARENB;
            tty.c_iflag &= ~INPCK;
            break;
        case 'E': case 'e':
            tty.c_cflag |= PARENB;   // even parity
            tty.c_cflag &= ~PARODD;
            tty.c_iflag |= INPCK;
            break;
        case 'O': case 'o':
            tty.c_cflag |= PARENB;   // odd parity
            tty.c_cflag |= PARODD;
            tty.c_iflag |= INPCK;
            break;
        default:
            log::error("serial: invalid parity");
            return -1;
    }

    // Stop bits.
    if (stopBits == 2) {
        tty.c_cflag |= CSTOPB;
    } else if (stopBits == 1) {
        tty.c_cflag &= ~CSTOPB;
    } else {
        log::error("serial: invalid stop bits " + std::to_string(stopBits));
        return -1;
    }

    // Raw mode: no output processing, no line editing, no software flow control.
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        log::error(std::string("serial: tcsetattr failed — ") + std::strerror(errno));
        return -1;
    }
    return 0;
}

ssize_t SerialPort::write(const std::string& data) {
    return write(reinterpret_cast<const unsigned char*>(data.data()), data.size());
}

ssize_t SerialPort::write(const unsigned char* data, std::size_t len) {
    if (fd_ < 0 || data == nullptr || len == 0) return -1;
    return ::write(fd_, data, len);
}

int SerialPort::available() const {
    if (fd_ < 0) return 0;
    int n = 0;
    if (::ioctl(fd_, FIONREAD, &n) < 0) return 0;
    return n;
}

ssize_t SerialPort::read(unsigned char* buf, std::size_t max) {
    if (fd_ < 0 || buf == nullptr || max == 0) return -1;
    return ::read(fd_, buf, max);
}

} // namespace r3d::serial
