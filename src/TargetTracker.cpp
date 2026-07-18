#include "TargetTracker.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace rm_gimbal {
namespace {

float lerp(float previous, float current, double alpha) {
    return static_cast<float>(previous * (1.0 - alpha) + current * alpha);
}

cv::Point2f lerpPoint(const cv::Point2f& previous, const cv::Point2f& current, double alpha) {
    return {lerp(previous.x, current.x, alpha), lerp(previous.y, current.y, alpha)};
}

cv::Size2f lerpSize(const cv::Size2f& previous, const cv::Size2f& current, double alpha) {
    return {lerp(previous.width, current.width, alpha), lerp(previous.height, current.height, alpha)};
}

// PnP 要求图像角点顺序稳定，这里按左上、右上、右下、左下排序。
std::vector<cv::Point2f> orderPoints(const cv::RotatedRect& rect) {
    cv::Point2f points[4];
    rect.points(points);

    std::vector<cv::Point2f> ordered(points, points + 4);
    std::sort(ordered.begin(), ordered.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {
        return lhs.y == rhs.y ? lhs.x < rhs.x : lhs.y < rhs.y;
    });

    std::vector<cv::Point2f> top{ordered[0], ordered[1]};
    std::vector<cv::Point2f> bottom{ordered[2], ordered[3]};
    std::sort(top.begin(), top.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {
        return lhs.x < rhs.x;
    });
    std::sort(bottom.begin(), bottom.end(), [](const cv::Point2f& lhs, const cv::Point2f& rhs) {
        return lhs.x < rhs.x;
    });

    return {top[0], top[1], bottom[1], bottom[0]};
}

}  // namespace

TargetTracker::TargetTracker(TargetTrackerConfig config) : config_(std::move(config)) {}

std::optional<BeaconTarget> TargetTracker::update(const std::optional<BeaconTarget>& measurement) {
    if (!measurement.has_value() || measurement->confidence < config_.min_confidence) {
        ++lost_frames_;

        // 信标短暂漏检时继续输出上一帧目标，避免绿色框和 PnP 结果突然消失。
        if (tracked_target_.has_value() && lost_frames_ <= config_.max_lost_frames) {
            BeaconTarget held = *tracked_target_;
            held.confidence *= 0.85;
            tracked_target_ = held;
            return tracked_target_;
        }

        reset();
        return std::nullopt;
    }

    lost_frames_ = 0;
    if (!tracked_target_.has_value()) {
        tracked_target_ = *measurement;
        return tracked_target_;
    }

    tracked_target_ = smoothTarget(*tracked_target_, *measurement);
    return tracked_target_;
}

void TargetTracker::reset() noexcept {
    tracked_target_.reset();
    lost_frames_ = 0;
}

bool TargetTracker::hasTarget() const noexcept {
    return tracked_target_.has_value();
}

int TargetTracker::lostFrames() const noexcept {
    return lost_frames_;
}

const TargetTrackerConfig& TargetTracker::config() const noexcept {
    return config_;
}

BeaconTarget TargetTracker::smoothTarget(const BeaconTarget& previous,
                                         const BeaconTarget& measurement) const {
    const double alpha = std::clamp(config_.smooth_alpha, 0.0, 1.0);

    BeaconTarget smoothed = measurement;
    smoothed.bounding_rect.center = lerpPoint(previous.bounding_rect.center,
                                             measurement.bounding_rect.center,
                                             alpha);
    smoothed.bounding_rect.size = lerpSize(previous.bounding_rect.size,
                                          measurement.bounding_rect.size,
                                          alpha);
    smoothed.bounding_rect.angle = smoothAngle(previous.bounding_rect.angle,
                                              measurement.bounding_rect.angle);
    smoothed.center = smoothed.bounding_rect.center;
    smoothed.image_points = orderPoints(smoothed.bounding_rect);

    // 置信度也平滑一下，显示和控制状态不会因为单帧分数变化而抖。
    smoothed.confidence = previous.confidence * (1.0 - alpha) + measurement.confidence * alpha;
    return smoothed;
}

float TargetTracker::smoothAngle(float previous_deg, float measurement_deg) const {
    const double alpha = std::clamp(config_.smooth_alpha, 0.0, 1.0);
    float diff = measurement_deg - previous_deg;

    // minAreaRect 的角度在边界处可能跳变，这里选最短角度差做平滑。
    while (diff > 90.0F) {
        diff -= 180.0F;
    }
    while (diff < -90.0F) {
        diff += 180.0F;
    }

    return previous_deg + static_cast<float>(diff * alpha);
}

}  // namespace rm_gimbal
