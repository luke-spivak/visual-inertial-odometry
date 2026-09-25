// Feature snapshots are serialized for the shared recording worker.
#pragma once
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <vector>

struct FeaturePoint {
    size_t id;
    float x, y;
};
struct FeatureFrame {
    int64_t timestamp_ns;
    size_t frame_index;
    int width, height;
    bool initialized;
    std::vector<FeaturePoint> points;
};

inline std::string feature_record(const FeatureFrame& frame) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(6);
    out << "{\"type\":\"frame\",\"timestamp_ns\":" << frame.timestamp_ns
        << ",\"frame_index\":" << frame.frame_index << ",\"camera_id\":0,\"width\":" << frame.width
        << ",\"height\":" << frame.height
        << ",\"initialized\":" << (frame.initialized ? "true" : "false") << ",\"features\":[";
    for (size_t i = 0; i < frame.points.size(); ++i) {
        const auto& point = frame.points[i];
        out << (i ? "," : "") << '[' << point.id << ',' << point.x << ',' << point.y << ']';
    }
    out << "]}\n";
    return out.str();
}
