#include "serial_port.h"

#include <cerrno>
#include <fcntl.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <system_error>
#include <termios.h>
#include <unistd.h>

namespace vio {
namespace {
speed_t baud_rate(unsigned baud) {
    switch (baud) {
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
#ifdef B460800
    case 460800:
        return B460800;
#endif
#ifdef B921600
    case 921600:
        return B921600;
#endif
    default:
        throw std::invalid_argument("unsupported UART baud rate");
    }
}

std::size_t transferred(ssize_t count) {
    if (count >= 0)
        return static_cast<std::size_t>(count);
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        return 0;
    throw std::system_error(errno, std::generic_category(), "UART I/O");
}
} // namespace

SerialPort::SerialPort(const std::string& path, unsigned baud) {
    const auto speed = baud_rate(baud);
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0)
        throw std::system_error(errno, std::generic_category(), "open UART " + path);
    try {
        if (::ioctl(fd_, TIOCEXCL) < 0)
            throw std::system_error(errno, std::generic_category(), "exclusive UART access");
        termios settings{};
        if (::tcgetattr(fd_, &settings) < 0)
            throw std::system_error(errno, std::generic_category(), "read UART settings");
        ::cfmakeraw(&settings);
        settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        settings.c_cflag |= CS8 | CLOCAL | CREAD;
        settings.c_cc[VMIN] = 1;
        settings.c_cc[VTIME] = 0;
        if (::cfsetispeed(&settings, speed) < 0 || ::cfsetospeed(&settings, speed) < 0 ||
            ::tcsetattr(fd_, TCSANOW, &settings) < 0 || ::tcflush(fd_, TCIOFLUSH) < 0) {
            throw std::system_error(errno, std::generic_category(), "configure UART");
        }
    } catch (...) {
        ::close(fd_);
        fd_ = -1;
        throw;
    }
}

SerialPort::~SerialPort() {
    if (fd_ >= 0) {
        // Discard queued bytes on shutdown; do not drain stale measurements.
        ::tcflush(fd_, TCOFLUSH);
        ::close(fd_);
    }
}

std::size_t SerialPort::read(std::uint8_t* data, std::size_t capacity) {
    if (capacity == 0)
        return 0;
    const auto count = ::read(fd_, data, capacity);
    if (count == 0)
        throw std::runtime_error("UART disconnected");
    return transferred(count);
}

std::size_t SerialPort::write(const std::uint8_t* data, std::size_t size) {
    return transferred(::write(fd_, data, size));
}

} // namespace vio
