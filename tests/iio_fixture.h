#pragma once
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

namespace fs = std::filesystem;

inline void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}
inline void put(const fs::path& p, const std::string& value) {
    fs::create_directories(p.parent_path());
    std::ofstream(p) << value;
}
inline std::string get(const fs::path& p) {
    std::ifstream f(p);
    std::string s;
    f >> s;
    return s;
}

struct Fixture {
    fs::path root;
    Fixture() {
        char name[] = "/tmp/vio-iio-XXXXXX";
        auto result = ::mkdtemp(name);
        require(result, "temporary directory");
        root = result;
        for (int i = 0; i < 2; ++i) {
            auto d = device(i);
            std::string pre = i == 0 ? "in_accel_" : "in_anglvel_";
            put(d / "name", i == 0 ? "lsm6dsx_accel" : "lsm6dsx_gyro");
            put(d / "sampling_frequency", "416");
            put(d / "current_timestamp_clock", "realtime");
            put(d / (pre + "scale_available"), i == 0 ? "0.000598 0.004788" : "0.000153 0.001065");
            put(d / (pre + "scale"), i == 0 ? "0.004788" : "0.001065");
            put(d / "buffer0/enable", "0");
            put(d / "buffer0/length", "128");
            put(d / "buffer0/watermark", "1");
            for (int c = 0; c < 4; ++c) {
                auto base = d / "scan_elements" / (c == 3 ? "in_timestamp" : pre + "xyz"[c]);
                put(base.string() + "_en", "0");
                put(base.string() + "_index", std::to_string(c));
                put(base.string() + "_type", c == 3 ? "le:s64/64>>0" : "le:s16/16>>0");
            }
            for (char a : std::string("xyz"))
                put(d / (pre + a + "_raw"), a == 'z' ? "2048" : "0");
        }
    }
    fs::path device(int i) const {
        return root / ("iio:device" + std::to_string(i));
    }
    ~Fixture() {
        std::error_code error;
        fs::remove_all(root, error);
    }
    void disabled() {
        for (int i = 0; i < 2; ++i)
            require(get(device(i) / "buffer0/enable") == "0", "buffers disabled");
    }
};
