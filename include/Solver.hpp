#pragma once

#include "Detector.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <optional>
#include <vector>

namespace rm_gimbal {

struct CameraCalibration {
    cv::Mat camera_matrix{};
    cv::Mat distortion_coeffs{};
};

struct SolverConfig {
    CameraCalibration calibration{};
    cv::Size2f beacon_size_mm{50.0F, 67.0F};
    double bullet_speed_mps{18.0};
    double gravity_mps2{9.80665};
    bool enable_gravity_compensation{true};
};

struct PoseResult {
    cv::Vec3d rotation_vector{};
    cv::Vec3d translation_vector{};
    double yaw_deg{0.0};
    double pitch_deg{0.0};
    double distance_m{0.0};
};

class BallisticSolver {
public:
    explicit BallisticSolver(SolverConfig config = {});
    ~BallisticSolver() = default;

    BallisticSolver(const BallisticSolver&) = default;
    BallisticSolver& operator=(const BallisticSolver&) = default;
    BallisticSolver(BallisticSolver&&) noexcept = default;
    BallisticSolver& operator=(BallisticSolver&&) noexcept = default;

    [[nodiscard]] std::optional<PoseResult> solve(const ArmorTarget& target) const;
    [[nodiscard]] std::vector<cv::Point3f> buildObjectPoints(bool large_armor = false) const;
    void setCalibration(const CameraCalibration& calibration);

    void setBulletSpeed(double speed_mps) noexcept;
    [[nodiscard]] double bulletSpeed() const noexcept;
    [[nodiscard]] const SolverConfig& config() const noexcept;

private:
    [[nodiscard]] double compensatePitch(double pitch_rad, double distance_m) const;

    SolverConfig config_;
};

}  // namespace rm_gimbal
