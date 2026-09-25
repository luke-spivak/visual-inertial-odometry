#include "serial_port.h"

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <system_error>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {
void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

struct PseudoTerminal {
    int master{-1};
    std::string path;
    PseudoTerminal() {
        master = ::posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
        require(master >= 0, "open pseudoterminal");
        if (::grantpt(master) != 0 || ::unlockpt(master) != 0 || ::ptsname(master) == nullptr) {
            ::close(master);
            throw std::runtime_error("prepare pseudoterminal");
        }
        path = ::ptsname(master);
    }
    ~PseudoTerminal() {
        if (master >= 0)
            ::close(master);
    }
};

void uart_round_trip() {
    PseudoTerminal terminal;
    vio::SerialPort port(terminal.path, 230400);
    std::array<std::uint8_t, 8> received{};
    require(port.read(received.data(), received.size()) == 0, "idle UART is nonblocking");
    // Raw mode must preserve NUL, newline, carriage return, XON/XOFF, and high bits.
    const std::array<std::uint8_t, 8> bytes{0, 10, 13, 17, 19, 127, 253, 255};
    require(::write(terminal.master, bytes.data(), bytes.size()) == ssize_t(bytes.size()),
            "fake controller write");
    std::size_t count = 0;
    for (int i = 0; i < 100 && count < bytes.size(); ++i) {
        count += port.read(received.data() + count, received.size() - count);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(count == bytes.size() && received == bytes, "raw UART receive");
    require(port.write(bytes.data(), bytes.size()) == bytes.size(), "UART write");
    count = 0;
    for (int i = 0; i < 100 && count < bytes.size(); ++i) {
        const auto n = ::read(terminal.master, received.data() + count, received.size() - count);
        if (n > 0)
            count += static_cast<std::size_t>(n);
        else
            require(n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK), "fake controller read");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    require(count == bytes.size() && received == bytes, "raw UART transmit");
    ::close(terminal.master);
    terminal.master = -1;
    bool disconnected = false;
    try {
        port.read(received.data(), received.size());
    } catch (const std::runtime_error&) {
        disconnected = true;
    }
    require(disconnected, "UART hangup reported");
}

void startup_errors() {
    bool failed = false;
    try {
        vio::SerialPort port("/nonexistent-vio-uart", 230400);
    } catch (const std::system_error&) {
        failed = true;
    }
    require(failed, "missing device rejected");
    failed = false;
    try {
        vio::SerialPort port("/nonexistent-vio-uart", 12345);
    } catch (const std::invalid_argument&) {
        failed = true;
    }
    require(failed, "unsupported baud rejected before opening");
}
} // namespace

int main() {
    try {
        startup_errors();
        uart_round_trip();
        std::cout << "POSIX UART checks passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
