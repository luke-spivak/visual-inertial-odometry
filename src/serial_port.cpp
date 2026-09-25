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
/// Translate a numeric baud rate into the constant expected by the OS.
/// Optional speeds are included only when the platform headers define them.
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

/// Interpret a read/write result: return the byte count, or zero to retry later.
/// On failure, errno still holds the error from the immediately preceding OS call;
/// this helper must not make another OS call before inspecting it.
std::size_t transferred(ssize_t count) {
    if (count >= 0)
        return static_cast<std::size_t>(count);
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
        return 0;
    throw std::system_error(errno, std::generic_category(), "UART I/O");
}
} // namespace

/// Open the device and configure a raw, nonblocking serial connection.
/// If any setup step fails, close the device before passing the error to the caller.
SerialPort::SerialPort(const std::string& path, unsigned baud) {
    const auto speed = baud_rate(baud);
    // Read/write access; no controlling terminal, no waiting, no inheritance on exec.
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0)
        throw std::system_error(errno, std::generic_category(), "open UART " + path);
    try {
        // Request exclusive access to prevent subsequent ordinary opens.
        if (::ioctl(fd_, TIOCEXCL) < 0)
            throw std::system_error(errno, std::generic_category(), "exclusive UART access");
        // Start with the current terminal settings, then disable text processing.
        termios settings{};
        if (::tcgetattr(fd_, &settings) < 0)
            throw std::system_error(errno, std::generic_category(), "read UART settings");
        ::cfmakeraw(&settings);
        // Clear old framing/flow-control flags; select 8 data bits, no parity,
        // one stop bit, ignore modem-control lines, and enable the receiver.
        settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
        settings.c_cflag |= CS8 | CLOCAL | CREAD;
        // Request at least one byte with no terminal read timer. O_NONBLOCK
        // still prevents waiting when no bytes are available.
        settings.c_cc[VMIN] = 1;
        settings.c_cc[VTIME] = 0;
        // Set both baud rates, apply immediately, and discard old buffered bytes.
        if (::cfsetispeed(&settings, speed) < 0 || ::cfsetospeed(&settings, speed) < 0 ||
            ::tcsetattr(fd_, TCSANOW, &settings) < 0 || ::tcflush(fd_, TCIOFLUSH) < 0) {
            throw std::system_error(errno, std::generic_category(), "configure UART");
        }
    } catch (...) {
        // The destructor will not run if construction fails; clean up here.
        ::close(fd_);
        fd_ = -1;
        throw;
    }
}

/// Discard bytes still waiting to be sent and release the device when this owner dies.
SerialPort::~SerialPort() {
    if (fd_ >= 0) {
        // Discard queued bytes on shutdown; do not drain stale measurements.
        ::tcflush(fd_, TCOFLUSH);
        ::close(fd_);
    }
}

/// Copy available incoming bytes into the caller's buffer without waiting.
/// Return zero when there is nothing to read yet; report a disconnect as an exception.
std::size_t SerialPort::read(std::uint8_t* data, std::size_t capacity) {
    if (capacity == 0)
        return 0;
    const auto count = ::read(fd_, data, capacity);
    if (count == 0)
        throw std::runtime_error("UART disconnected");
    return transferred(count);
}

/// Give outgoing bytes to the OS without waiting for the wire to finish sending.
/// Return how many bytes it accepted; the caller retains and retries the remainder.
std::size_t SerialPort::write(const std::uint8_t* data, std::size_t size) {
    return transferred(::write(fd_, data, size));
}

} // namespace vio
