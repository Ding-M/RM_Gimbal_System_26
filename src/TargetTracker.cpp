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

        // 信标短暂漏检时用 Kalman 预测中心点，避免绿色框和 PnP 结果突然消失。
        if (tracked_target_.has_value() && lost_frames_ <= config_.max_lost_frames) {
            BeaconTarget held = *tracked_target_;
            if (kalman_initialized_) {
                held = withCenter(held, predictCenter());
            }
            held.confidence *= 0.85;
            tracked_target_ = held;
            return tracked_target_;
        }

        reset();
        return std::nullopt;
    }

    lost_frames_ = 0;
    if (!tracked_target_.has_value()) {
        initializeKalman(measurement->center);
        tracked_target_ = withCenter(*measurement, measurement->center);
        return tracked_target_;
    }

    tracked_target_ = smoothTarget(*tracked_target_, *measurement);
    return tracked_target_;
}

void TargetTracker::reset() noexcept {
    tracked_target_.reset();
    kalman_initialized_ = false;
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

void TargetTracker::initializeKalman(const cv::Point2f& center) {
    center_filter_ = cv::KalmanFilter(4, 2, 0, CV_32F);

    center_filter_.transitionMatrix = (cv::Mat_<float>(4, 4) << 1.0F, 0.0F, 1.0F, 0.0F,
                                                                0.0F, 1.0F, 0.0F, 1.0F,
                                                                0.0F, 0.0F, 1.0F, 0.0F,
                                                                0.0F, 0.0F, 0.0F, 1.0F);
    center_filter_.measurementMatrix = (cv::Mat_<float>(2, 4) << 1.0F, 0.0F, 0.0F, 0.0F,
                                                                 0.0F, 1.0F, 0.0F, 0.0F);

    cv::setIdentity(center_filter_.processNoiseCov, cv::Scalar(config_.kalman_process_noise));
    cv::setIdentity(center_filter_.measurementNoiseCov, cv::Scalar(config_.kalman_measurement_noise));
    cv::setIdentity(center_filter_.errorCovPost, cv::Scalar(config_.kalman_error_cov));

    center_filter_.statePost.at<float>(0) = center.x;
    center_filter_.statePost.at<float>(1) = center.y;
    center_filter_.statePost.at<float>(2) = 0.0F;
    center_filter_.statePost.at<float>(3) = 0.0F;
    kalman_initialized_ = true;
}

cv::Point2f TargetTracker::correctCenter(const cv::Point2f& measurement_center) {
    if (!kalman_initialized_) {
        initializeKalman(measurement_center);
    }

    center_filter_.predict();

    cv::Mat measurement(2, 1, CV_32F);
    measurement.at<float>(0) = measurement_center.x;
    measurement.at<float>(1) = measurement_center.y;

    const cv::Mat corrected = center_filter_.correct(measurement);
    return {corrected.at<float>(0), corrected.at<float>(1)};
}

cv::Point2f TargetTracker::predictCenter() {
    if (!kalman_initialized_) {
        return tracked_target_.has_value() ? tracked_target_->center : cv::Point2f{};
    }

    const cv::Mat prediction = center_filter_.predict();
    return {prediction.at<float>(0), prediction.at<float>(1)};
}

BeaconTarget TargetTracker::withCenter(BeaconTarget target, const cv::Point2f& center) const {
    target.bounding_rect.center = center;
    target.center = center;
    target.image_points = orderPoints(target.bounding_rect);
    return target;
}

BeaconTarget TargetTracker::smoothTarget(const BeaconTarget& previous,
                                         const BeaconTarget& measurement) {
    const double alpha = std::clamp(config_.smooth_alpha, 0.0, 1.0);

    BeaconTarget smoothed = measurement;
    smoothed.bounding_rect.center = correctCenter(measurement.bounding_rect.center);
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
