#include "process.h"

#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <spawn.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <system_error>
#include <thread>
#include <unistd.h>

extern char** environ;
namespace vio {
namespace fs = std::filesystem;

/// Spawn with a separate process group and reset signal disposition in the child.
ChildProcess::ChildProcess(const std::vector<std::string>& args, const fs::path& log) {
    if (args.empty())
        throw std::invalid_argument("empty child command");
    std::vector<char*> argv;
    for (const auto& arg : args)
        argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);
    posix_spawn_file_actions_t files;
    posix_spawnattr_t attributes;
    int result = posix_spawn_file_actions_init(&files);
    if (result)
        throw std::system_error(result, std::generic_category(), "spawn file actions");
    result = posix_spawnattr_init(&attributes);
    if (result) {
        posix_spawn_file_actions_destroy(&files);
        throw std::system_error(result, std::generic_category(), "spawn attributes");
    }
    try {
        auto check = [](int error) {
            if (error)
                throw std::system_error(error, std::generic_category(), "configure child process");
        };
        check(posix_spawn_file_actions_addopen(&files, STDIN_FILENO, "/dev/null", O_RDONLY, 0));
        check(posix_spawn_file_actions_addopen(&files, STDOUT_FILENO, "/dev/null", O_WRONLY, 0));
        check(posix_spawn_file_actions_addopen(&files, STDERR_FILENO, log.c_str(),
                                               O_WRONLY | O_CREAT | O_TRUNC, 0600));
        sigset_t empty, defaults;
        sigemptyset(&empty);
        sigemptyset(&defaults);
        sigaddset(&defaults, SIGINT);
        sigaddset(&defaults, SIGTERM);
        sigaddset(&defaults, SIGPIPE);
        check(posix_spawnattr_setsigmask(&attributes, &empty));
        check(posix_spawnattr_setsigdefault(&attributes, &defaults));
        check(posix_spawnattr_setpgroup(&attributes, 0));
        check(posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP | POSIX_SPAWN_SETSIGMASK |
                                                        POSIX_SPAWN_SETSIGDEF));
        pid_t child;
        check(
            posix_spawnp(&child, args.front().c_str(), &files, &attributes, argv.data(), environ));
        pid_ = child;
    } catch (...) {
        posix_spawnattr_destroy(&attributes);
        posix_spawn_file_actions_destroy(&files);
        throw;
    }
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&files);
}

/// Terminate and reap a still-running child during normal or exceptional cleanup.
ChildProcess::~ChildProcess() {
    try {
        stop(SIGTERM, std::chrono::milliseconds(250));
    } catch (...) {
    }
}

/// Check for exit without waiting; preserve the exit status for later callers.
std::optional<int> ChildProcess::poll() {
    if (result_)
        return result_;
    int status;
    auto result = ::waitpid(pid_, &status, WNOHANG);
    if (result < 0 && errno != EINTR)
        throw std::system_error(errno, std::generic_category(), "waitpid");
    if (result == pid_)
        result_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
    return result_;
}

/// Give the child a short grace period, then kill and reap it instead of waiting forever.
void ChildProcess::stop(int signal, std::chrono::milliseconds grace) {
    if (poll())
        return;
    ::kill(-pid_, signal);
    const auto deadline = std::chrono::steady_clock::now() + grace;
    while (!poll() && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (result_)
        return;
    ::kill(-pid_, SIGKILL);
    int status;
    while (::waitpid(pid_, &status, 0) < 0) {
        if (errno != EINTR)
            throw std::system_error(errno, std::generic_category(), "reap child");
    }
    result_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
}

/// Create both FIFOs before either camera output is opened, avoiding open-order deadlock.
CameraPipes::CameraPipes(const fs::path& parent) {
    std::string pattern = (parent / "camera-XXXXXX").string();
    char* path = ::mkdtemp(pattern.data());
    if (!path)
        throw std::system_error(errno, std::generic_category(), "create camera directory");
    directory_ = path;
    if (::mkfifo(frames().c_str(), 0600) || ::mkfifo(metadata().c_str(), 0600)) {
        const int error = errno;
        fs::remove_all(directory_);
        throw std::system_error(error, std::generic_category(), "create camera FIFO");
    }
}

/// Remove this capture's temporary paths after all readers and writers have stopped.
CameraPipes::~CameraPipes() {
    std::error_code error;
    fs::remove_all(directory_, error);
}

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
