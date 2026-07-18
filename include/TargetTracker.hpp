#pragma once

#include "Detector.hpp"

#include <optional>

namespace rm_gimbal {

struct TargetTrackerConfig {
    double min_confidence{0.35};
    double smooth_alpha{0.35};
    int max_lost_frames{5};
};

// 对检测到的信标目标做时序稳定：
// 1. 低置信度目标不立刻采用；
// 2. 中心、尺寸和角度做低通滤波；
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
    [[nodiscard]] BeaconTarget smoothTarget(const BeaconTarget& previous,
                                            const BeaconTarget& measurement) const;
    [[nodiscard]] float smoothAngle(float previous_deg, float measurement_deg) const;

    TargetTrackerConfig config_;
    std::optional<BeaconTarget> tracked_target_{};
    int lost_frames_{0};
};

}  // namespace rm_gimbal
