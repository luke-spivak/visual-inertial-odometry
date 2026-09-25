// Native capture/estimator application. Service cutover is a separate migration gate.
#include "flight_application.h"

#include <csignal>
#include <cstdlib>
#include <iostream>
#include <pwd.h>
#include <unistd.h>

namespace {
volatile std::sig_atomic_t stopping = 0;
/// Signal handlers only set a flag; normal code performs I/O and cleanup.
void stop(int) {
    stopping = 1;
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2) {
            std::cerr << "usage: vio_flight /path/to/flight.json\n";
            return 2;
        }
        passwd* user = nullptr;
        if (const char* name = std::getenv("SUDO_USER"))
            user = ::getpwnam(name);
        if (!user)
            user = ::getpwuid(::getuid());
        if (!user)
            throw std::runtime_error("cannot determine operating user's home");
        const auto config = vio::load_flight_config(argv[1], user->pw_dir);
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);
        vio::run_flight(
            config, [&] { return std::make_unique<vio::SerialPort>(config.device, config.baud); },
            [&] { return vio::make_openvins_runner(config.estimator_config, config.verbosity); },
            [] { return stopping != 0; });
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "vio_flight: " << error.what() << '\n';
        return 1;
    }
}
