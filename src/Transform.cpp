#include "Transform.hpp"

#include <cmath>
#include <utility>

namespace rm_gimbal {
namespace {

double degToRad(double deg) {
    return deg * CV_PI / 180.0;
}

cv::Matx33d rotationYaw(double yaw_rad) {
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);

    // Rotate around world/gimbal z axis.
    return {
        c, -s, 0.0,
        s, c, 0.0,
        0.0, 0.0, 1.0,
    };
}

cv::Matx33d rotationPitch(double pitch_rad) {
    const double c = std::cos(pitch_rad);
    const double s = std::sin(pitch_rad);

    // Rotate around gimbal y axis.
    return {
        c, 0.0, s,
        0.0, 1.0, 0.0,
        -s, 0.0, c,
    };
}

}  // namespace

CoordinateTransformer::CoordinateTransformer(TransformConfig config) : config_(std::move(config)) {}

cv::Vec3d CoordinateTransformer::cameraToGimbal(const cv::Vec3d& point_camera_mm) const {
    // 坐标轴转换：
    // camera x(右) -> gimbal -y(左的反方向)
    // camera y(下) -> gimbal -z(上的反方向)
    // camera z(前) -> gimbal  x(前)
    const cv::Vec3d point_gimbal{
        point_camera_mm[2],
        -point_camera_mm[0],
        -point_camera_mm[1],
    };

    return point_gimbal + config_.camera_to_gimbal_translation_mm;
}

cv::Vec3d CoordinateTransformer::gimbalToWorld(const cv::Vec3d& point_gimbal_mm,
                                               const GimbalState& state) const {
    const cv::Matx33d rotation = rotationYaw(degToRad(state.yaw_deg)) *
                                 rotationPitch(degToRad(state.pitch_deg));
    return rotation * point_gimbal_mm;
}

CoordinateResult CoordinateTransformer::cameraToWorld(const cv::Vec3d& point_camera_mm,
                                                      const GimbalState& state) const {
    CoordinateResult result;
    result.camera_mm = point_camera_mm;
    result.gimbal_mm = cameraToGimbal(point_camera_mm);
    result.world_mm = gimbalToWorld(result.gimbal_mm, state);
    return result;
}

void CoordinateTransformer::setConfig(const TransformConfig& config) noexcept {
    config_ = config;
}

const TransformConfig& CoordinateTransformer::config() const noexcept {
    return config_;
}

}  // namespace rm_gimbal
