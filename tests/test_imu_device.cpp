#include "imu_device.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <unistd.h>

#include "iio_fixture.h"

int main() {
    try {
        Fixture valid;
        {
            vio::ImuDevices devices(8, valid.root, valid.root);
            for (const auto& d : devices.devices()) {
                require(d.record_bytes == 16 && d.timestamp_offset == 8 &&
                            d.axis_offsets == std::array<int, 3>{0, 2, 4},
                        "aligned scan layout");
                require(get(d.sysfs / "current_timestamp_clock") == "monotonic",
                        "monotonic selected");
                require(get(d.sysfs / "buffer0/enable") == "1", "buffer enabled");
            }
            devices.save_metadata(valid.root / "capture");
            nlohmann::json metadata;
            std::ifstream(valid.root / "capture.imu.json") >> metadata;
            require(metadata["devices"]["gyro"]["channels"].size() == 4, "replay metadata");
        }
        valid.disabled();
        for (int failure = 0; failure < 4; ++failure) {
            Fixture fixture;
            if (failure == 0)
                put(fixture.device(1) / "scan_elements/in_timestamp_type", "be:s64/64>>0");
            if (failure == 1)
                fs::remove(fixture.device(1) / "current_timestamp_clock");
            if (failure == 2)
                put(fixture.device(0) / "in_accel_z_raw", "0");
            if (failure == 3)
                put(fixture.device(1) / "scan_elements/in_anglvel_z_index", "1");
            bool rejected = false;
            try {
                vio::ImuDevices devices(8, fixture.root, fixture.root);
            } catch (const std::exception&) {
                rejected = true;
            }
            require(rejected, "bad sensor configuration rejected");
            fixture.disabled();
        }
        Fixture busy;
        put(busy.device(0) / "buffer0/enable", "1");
        bool rejected = false;
        try {
            vio::ImuDevices devices(8, busy.root, busy.root);
        } catch (const std::exception&) {
            rejected = true;
        }
        require(rejected && get(busy.device(0) / "buffer0/enable") == "1",
                "do not take over another capture");
        std::cout << "IIO setup and cleanup checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
