#ifndef RM_GIMBAL_SOLVER_HPP
#define RM_GIMBAL_SOLVER_HPP

#include <optional>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

// 引入 Detector.hpp 以获取 ArmorTarget (BeaconTarget) 的定义
#include "Detector.hpp"

namespace rm_gimbal {

// 确保在本头文件中完整定义所需的结构体，避免找不到类型
struct CameraCalibration {
    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
};

struct SolverConfig {
    CameraCalibration calibration;
    bool enable_gravity_compensation = true;
    double bullet_speed_mps = 25.0;
    double gravity_mps2 = 9.80665;
    cv::Size2f beacon_size_mm{135.0f, 55.0f};
};

struct PoseResult {
    cv::Mat translation_vector;
    cv::Mat rotation_vector;
    double distance_m = 0.0;
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
};

class BallisticSolver {
public:
    explicit BallisticSolver(SolverConfig config);
    
    std::optional<PoseResult> solve(const ArmorTarget& target) const;
    void setCalibration(const CameraCalibration& calibration);
    void setBulletSpeed(double speed_mps) noexcept;
    double bulletSpeed() const noexcept;
    const SolverConfig& config() const noexcept;
    double compensatePitch(double pitch_rad, double distance_m) const;

private:
    std::vector<cv::Point3f> buildObjectPoints(bool large_armor = false) const;
    
    SolverConfig config_;
};

} // namespace rm_gimbal

#endif // RM_GIMBAL_SOLVER_HPP