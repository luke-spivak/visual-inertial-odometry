#include "estimator_runner.h"
#include "iio_fixture.h"
#include "process.h"

#include <iostream>

int main(int argc, char** argv) {
    try {
        require(argc == 2, "estimator configuration required");
        Fixture fixture;
        vio::CameraPipes pipes(fixture.root);
        auto runner = vio::make_openvins_runner(argv[1], "SILENT");
        Eigen::Matrix3d expected;
        expected << -0.999918663, -0.012696196, 0.001214223, -0.012669127, 0.999716201, 0.020174511,
            -0.001470011, 0.020157486, -0.99979574;
        require((runner->camera_from_imu() - expected).norm() < 1e-6,
                "calibration direction matches camera-from-IMU");
        vio::RunnerOptions options;
        options.frames = pipes.frames();
        options.metadata = pipes.metadata();
        options.imu_lpf_hz = 0;
        for (const auto* kind : {"accel", "gyro"}) {
            vio::ImuDevice device;
            device.kind = kind;
            device.chardev = fixture.root / (std::string(kind) + ".bin");
            device.record_bytes = 16;
            device.timestamp_offset = 8;
            device.axis_offsets = {0, 2, 4};
            device.scale = 0.001;
            put(device.chardev, "");
            options.devices.push_back(device);
        }
        // Empty sensor streams trigger a worker exception while both camera FIFOs
        // have no writers. All four workers must still terminate and unwind.
        std::atomic<bool> stop{false};
        vio::LatestEstimate estimates;
        bool failed = false;
        try {
            runner->run(options, stop, estimates);
        } catch (const std::runtime_error& error) {
            failed = std::string(error.what()).find("IMU stream ended") != std::string::npos;
        }
        require(failed && stop && !estimates.load(),
                "worker error propagated with coordinated cleanup");
        std::cout << "OpenVINS calibration and worker cleanup checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
