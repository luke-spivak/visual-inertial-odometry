// Standalone test: c++ -std=c++17 -pthread -I src/openvins_runner this.cpp -o test
#include "feature_log.h"
#include <cassert>
#include <chrono>
#include <fstream>
int main(int argc, char** argv) {
    assert(argc == 2);
    {
        FeatureLog log;
        log.start(argv[1]);
        log.push({1234567890123456789LL, 42, 320, 160, false, {{7, 100, 110}}});
        log.close();
        std::ifstream f(argv[1]);
        std::string all((std::istreambuf_iterator<char>(f)), {});
        assert(all.find("1234567890123456789") != std::string::npos);
        assert(all.find("[7,100.000000,110.000000]") != std::string::npos);
        assert(all.find("\"type\":\"end\"") != std::string::npos);
    }
    {
        FeatureLog log;
        log.start("/dev/full");
        for (int i = 0; i < 100 && log.active(); ++i) {
            log.push({i, size_t(i), 320, 160, false, {}});
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        assert(!log.active());
        log.close();
    }
}
