// Exclusive runtime ownership and persistent MAVLink reset counters.
#include "session_store.h"
#include <cerrno>
#include <fcntl.h>
#include <fstream>
#include <stdexcept>
#include <sys/file.h>
#include <system_error>
#include <unistd.h>

namespace vio {
namespace fs = std::filesystem;

/// Lock the runtime directory before opening UART or changing IMU settings.
SessionStore::SessionStore(const fs::path& directory) : directory_(directory) {
    fs::create_directories(directory_);
    lock_ = ::open((directory_ / "application.lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (lock_ < 0)
        throw std::system_error(errno, std::generic_category(), "open application lock");
    if (::flock(lock_, LOCK_EX | LOCK_NB) < 0) {
        const int error = errno;
        ::close(lock_);
        lock_ = -1;
        throw std::system_error(error, std::generic_category(),
                                "another native application owns this runtime directory");
    }
}

/// Release the process lock without deleting the counter used by the next process.
SessionStore::~SessionStore() {
    if (lock_ >= 0)
        ::close(lock_);
}

/// Persist the next byte before a session can transmit, so crashes cannot reuse it.
unsigned char SessionStore::reserve_reset() {
    const auto path = directory_ / "reset_counter";
    unsigned next = 0;
    if (fs::exists(path)) {
        std::ifstream input(path);
        unsigned previous;
        std::string extra;
        if (!(input >> previous) || previous > 255 || (input >> extra))
            throw std::runtime_error("invalid reset counter file");
        next = (previous + 1) % 256;
    }
    const auto temporary = directory_ / "reset_counter.new";
    int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
    if (fd < 0)
        throw std::system_error(errno, std::generic_category(), "open reset counter");
    const auto text = std::to_string(next) + "\n";
    bool ok = ::write(fd, text.data(), text.size()) == static_cast<ssize_t>(text.size()) &&
              ::fsync(fd) == 0;
    ::close(fd);
    if (!ok)
        throw std::runtime_error("cannot persist reset counter");
    fs::rename(temporary, path);
    fd = ::open(directory_.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0)
        throw std::runtime_error("cannot open reset counter directory");
    ok = ::fsync(fd) == 0;
    ::close(fd);
    if (!ok)
        throw std::runtime_error("cannot sync reset counter directory");
    return static_cast<unsigned char>(next);
}
} // namespace vio
