#pragma once

#include <filesystem>

namespace vio {
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
