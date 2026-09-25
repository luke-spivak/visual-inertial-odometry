#include "estimator_runner.h"
#include "iio_fixture.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>

#include <iostream>
#include <nlohmann/json.hpp>

int main(int argc, char** argv) {
    try {
        require(argc == 2, "estimator configuration required");
        Fixture fixture;
        require(::mkfifo((fixture.root / "frames").c_str(), 0600) == 0, "frame FIFO");
        require(::mkfifo((fixture.root / "metadata").c_str(), 0600) == 0, "metadata FIFO");
        auto runner = vio::make_openvins_runner(argv[1], "SILENT");
        Eigen::Matrix3d expected;
        expected << -0.999918663, -0.012696196, 0.001214223, -0.012669127, 0.999716201, 0.020174511,
            -0.001470011, 0.020157486, -0.99979574;
        require((runner->camera_from_imu() - expected).norm() < 1e-6,
                "calibration direction matches camera-from-IMU");
        vio::RunnerOptions options;
        options.frames = fixture.root / "frames";
        options.metadata = fixture.root / "metadata";
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
        // Keep sensor FIFOs open but idle to exercise native camera ingestion/error propagation.
        struct Camera final : vio::CameraSource {
            unsigned reads{0};
            void start(double, std::optional<vio::Exposure>) override {}
            void stop() override {}
            std::optional<vio::CameraFrame> read(std::chrono::milliseconds) override {
                if (++reads > 1)
                    throw std::runtime_error("injected native camera failure");
                return vio::CameraFrame{
                    std::vector<std::uint8_t>(1280 * 800, 42), 1000000000, 17, {200, 1}};
            }
        } camera;
        std::vector<int> keep_open;
        for (auto& device : options.devices) {
            fs::remove(device.chardev);
            require(::mkfifo(device.chardev.c_str(), 0600) == 0, "sensor FIFO");
            keep_open.push_back(::open(device.chardev.c_str(), O_RDWR | O_NONBLOCK));
            require(keep_open.back() >= 0, "hold sensor FIFO open");
        }
        runner = vio::make_openvins_runner(argv[1], "SILENT");
        options.camera = &camera;
        options.recording_prefix = fixture.root / "native";
        stop = false;
        failed = false;
        try {
            runner->run(options, stop, estimates);
        } catch (const std::runtime_error& error) {
            failed = std::string(error.what()) == "injected native camera failure";
        }
        for (int fd : keep_open)
            ::close(fd);
        require(failed && stop, "native camera error propagates and stops IMU workers");
        require(fs::file_size(fixture.root / "native.y16") == 1280 * 800 * 2,
                "native frame recorded");
        std::ifstream raw(fixture.root / "native.y16", std::ios::binary);
        char bytes[2];
        raw.read(bytes, 2);
        require(bytes[0] == 0 && bytes[1] == 42, "legacy replay pixel layout preserved");
        nlohmann::json metadata;
        std::ifstream(fixture.root / "native.meta.json") >> metadata;
        require(metadata[0]["SensorTimestamp"] == 1000000000 && metadata[0]["Sequence"] == 17,
                "native frame and metadata stay paired");
        std::cout << "OpenVINS calibration and worker cleanup checks passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
