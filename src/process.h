#pragma once

#include <chrono>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace vio {

/// Own one child process group. Arguments go directly to exec, never through a shell.
class ChildProcess {
public:
    ChildProcess(const std::vector<std::string>& arguments, const std::filesystem::path& log);
    ~ChildProcess();
    ChildProcess(const ChildProcess&) = delete;
    ChildProcess& operator=(const ChildProcess&) = delete;
    std::optional<int> poll();
    void stop(int signal, std::chrono::milliseconds grace);

private:
    int pid_{-1};
    std::optional<int> result_;
};

/// Temporary camera FIFO directory; destruction removes only this capture's files.
class CameraPipes {
public:
    explicit CameraPipes(const std::filesystem::path& parent);
    ~CameraPipes();
    CameraPipes(const CameraPipes&) = delete;
    CameraPipes& operator=(const CameraPipes&) = delete;
    std::filesystem::path frames() const {
        return directory_ / "frames";
    }
    std::filesystem::path metadata() const {
        return directory_ / "metadata";
    }

private:
    std::filesystem::path directory_;
};

/// Exclusive application lock and crash-safe reservation of MAVLink reset bytes.
class SessionStore {
public:
    explicit SessionStore(const std::filesystem::path& directory);
    ~SessionStore();
    SessionStore(const SessionStore&) = delete;
    SessionStore& operator=(const SessionStore&) = delete;
    unsigned char reserve_reset();

private:
    std::filesystem::path directory_;
    int lock_{-1};
};
} // namespace vio
