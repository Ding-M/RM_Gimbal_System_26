#include "Solver.hpp"

#include <cmath>
#include <utility>

namespace rm_gimbal {
namespace {

constexpr double kRadiansToDegrees = 180.0 / CV_PI;

// PnP 必须依赖真实相机内参；没有 3x3 camera_matrix 时直接不解算。
bool hasCalibration(const CameraCalibration& calibration) {
    return !calibration.camera_matrix.empty() &&
           calibration.camera_matrix.rows == 3 &&
           calibration.camera_matrix.cols == 3;
}

}  // namespace

BallisticSolver::BallisticSolver(SolverConfig config) : config_(std::move(config)) {}

// 使用信标外接矩形的 4 个图像点和真实尺寸，解算目标相对相机的三维位姿。
std::optional<PoseResult> BallisticSolver::solve(const ArmorTarget& target) const {
    if (!hasCalibration(config_.calibration) || target.image_points.size() != 4) {
        return std::nullopt;
    }

    cv::Vec3d rotation_vector;
    cv::Vec3d translation_vector;
    // IPPE 适合平面矩形目标；这里把信标正面外框近似成一个 50mm x 67mm 平面。
    const bool ok = cv::solvePnP(buildObjectPoints(),
                                 target.image_points,
                                 config_.calibration.camera_matrix,
                                 config_.calibration.distortion_coeffs,
                                 rotation_vector,
                                 translation_vector,
                                 false,
                                 cv::SOLVEPNP_IPPE);
    if (!ok) {
        return std::nullopt;
    }

    const double x_mm = translation_vector[0];
    const double y_mm = translation_vector[1];
    const double z_mm = translation_vector[2];

    // OpenCV 相机坐标系：x 向右，y 向下，z 向前。yaw/pitch 先在相机坐标系下计算。
    PoseResult result;
    result.rotation_vector = rotation_vector;
    result.translation_vector = translation_vector;
    result.distance_m = cv::norm(translation_vector) / 1000.0;
    result.yaw_deg = std::atan2(x_mm, z_mm) * kRadiansToDegrees;

    const double horizontal_distance = std::hypot(x_mm, z_mm);
    result.pitch_deg = std::atan2(-y_mm, horizontal_distance) * kRadiansToDegrees;
    return result;
}

// 构造信标外框在目标坐标系下的四个 3D 点，顺序与 Detector 输出的图像点保持一致：
// 左上、右上、右下、左下。
std::vector<cv::Point3f> BallisticSolver::buildObjectPoints(bool large_armor) const {
    (void)large_armor;

    const float half_width = config_.beacon_size_mm.width * 0.5F;
    const float half_height = config_.beacon_size_mm.height * 0.5F;

    return {
        {-half_width, -half_height, 0.0F},
        {half_width, -half_height, 0.0F},
        {half_width, half_height, 0.0F},
        {-half_width, half_height, 0.0F},
    };
}

// 标定完成后，主程序用这个接口把 camera_matrix 和 distortion_coeffs 注入解算器。
void BallisticSolver::setCalibration(const CameraCalibration& calibration) {
    config_.calibration = calibration;
}

// 预留给后续弹道补偿或云台控制使用，目前信标跟踪阶段暂不依赖弹速。
void BallisticSolver::setBulletSpeed(double speed_mps) noexcept {
    config_.bullet_speed_mps = speed_mps;
}

double BallisticSolver::bulletSpeed() const noexcept {
    return config_.bullet_speed_mps;
}

const SolverConfig& BallisticSolver::config() const noexcept {
    return config_;
}

double BallisticSolver::compensatePitch(double pitch_rad, double distance_m) const {
    if (!config_.enable_gravity_compensation || config_.bullet_speed_mps <= 0.0) {
        return pitch_rad;
    }

    // 简化抛物线模型：根据飞行时间估算下坠角度，后续接弹丸时可再精细化。
    const double flight_time = distance_m / config_.bullet_speed_mps;
    const double drop_m = 0.5 * config_.gravity_mps2 * flight_time * flight_time;
    return pitch_rad + std::atan2(drop_m, distance_m);
}

}  // namespace rm_gimbal
