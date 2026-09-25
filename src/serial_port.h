#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace vio {

/// Nonblocking byte transport: zero means would-block; disconnects/errors throw.
/// Implementations may transfer fewer bytes than requested. One loop owns all I/O.
class ByteStream {
public:
    virtual ~ByteStream() = default;
    virtual std::size_t read(std::uint8_t* data, std::size_t capacity) = 0;
    virtual std::size_t write(const std::uint8_t* data, std::size_t size) = 0;
};

/// Exclusive POSIX UART owner, configured as raw 8N1 with no flow control.
/// Construction opens/configures the port; destruction closes it without draining.
class SerialPort final : public ByteStream {
public:
    SerialPort(const std::string& path, unsigned baud);
    ~SerialPort() override;
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;
    std::size_t read(std::uint8_t* data, std::size_t capacity) override;
    std::size_t write(const std::uint8_t* data, std::size_t size) override;

private:
    int fd_{-1};
};

} // namespace vio
