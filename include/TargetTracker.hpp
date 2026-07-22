#pragma once

#include "Detector.hpp"

#include <opencv2/video/tracking.hpp>

#include <optional>

namespace rm_gimbal {

struct TargetTrackerConfig {
    double min_confidence{0.35};
    double smooth_alpha{0.35};
    double kalman_process_noise{1e-2};
    double kalman_measurement_noise{5e-1};
    double kalman_error_cov{1.0};
    int max_lost_frames{5};
};

// 对检测到的信标目标做时序稳定：
// 1. 低置信度目标不立刻采用；
// 2. 中心点做 Kalman 滤波，尺寸和角度做低通滤波；
// 3. 短暂丢失时保留上一帧，避免 PnP 和画面显示突然跳变。
class TargetTracker {
public:
    explicit TargetTracker(TargetTrackerConfig config = {});

    [[nodiscard]] std::optional<BeaconTarget> update(const std::optional<BeaconTarget>& measurement);
    void reset() noexcept;

    [[nodiscard]] bool hasTarget() const noexcept;
    [[nodiscard]] int lostFrames() const noexcept;
    [[nodiscard]] const TargetTrackerConfig& config() const noexcept;

private:
    void initializeKalman(const cv::Point2f& center);
    [[nodiscard]] cv::Point2f correctCenter(const cv::Point2f& measurement_center);
    [[nodiscard]] cv::Point2f predictCenter();
    [[nodiscard]] BeaconTarget withCenter(BeaconTarget target, const cv::Point2f& center) const;
    [[nodiscard]] BeaconTarget smoothTarget(const BeaconTarget& previous,
                                            const BeaconTarget& measurement);
    [[nodiscard]] float smoothAngle(float previous_deg, float measurement_deg) const;

    TargetTrackerConfig config_;
    std::optional<BeaconTarget> tracked_target_{};
    cv::KalmanFilter center_filter_{4, 2, 0, CV_32F};
    bool kalman_initialized_{false};
    int lost_frames_{0};
};

}  // namespace rm_gimbal
