#pragma once

#include <opencv2/core.hpp>

namespace rm_gimbal {

struct GimbalState {
    double yaw_deg{0.0};
    double pitch_deg{0.0};
};

struct TransformConfig {
    cv::Vec3d camera_to_gimbal_translation_mm{0.0, 0.0, 0.0};
};

struct CoordinateResult {
    cv::Vec3d camera_mm{};
    cv::Vec3d gimbal_mm{};
    cv::Vec3d world_mm{};
};

class CoordinateTransformer {
public:
    explicit CoordinateTransformer(TransformConfig config = {});

    // OpenCV camera: x right, y down, z forward.
    // Gimbal: x forward, y left, z up.
    [[nodiscard]] cv::Vec3d cameraToGimbal(const cv::Vec3d& point_camera_mm) const;

    // World frame is obtained by rotating gimbal frame with current yaw/pitch.
    [[nodiscard]] cv::Vec3d gimbalToWorld(const cv::Vec3d& point_gimbal_mm,
                                          const GimbalState& state) const;

    [[nodiscard]] CoordinateResult cameraToWorld(const cv::Vec3d& point_camera_mm,
                                                 const GimbalState& state) const;

    void setConfig(const TransformConfig& config) noexcept;
    [[nodiscard]] const TransformConfig& config() const noexcept;

private:
    TransformConfig config_;
};

}  // namespace rm_gimbal
