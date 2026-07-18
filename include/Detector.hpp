#pragma once

#include <opencv2/core.hpp>

#include <optional>
#include <string>
#include <vector>

namespace rm_gimbal {

enum class EnemyColor {
    Red,
    Blue
};

struct DetectorConfig {
    EnemyColor enemy_color{EnemyColor::Red};
    double binary_threshold{80.0};
    double min_light_area{20.0};
    double min_light_aspect_ratio{0.25};
    double max_light_aspect_ratio{4.5};
    double min_rectangularity{0.45};
    int min_lights_per_beacon{2};
};

struct BeaconLight {
    cv::RotatedRect rect{};
    double area{0.0};
    double rectangularity{0.0};
};

struct BeaconTarget {
    cv::RotatedRect bounding_rect{};
    std::vector<cv::Point2f> image_points{};
    std::vector<BeaconLight> lights{};
    cv::Point2f center{};
    double confidence{0.0};
    std::string label{};
};

struct BeaconDebugResult {
    cv::Mat mask{};
    std::vector<BeaconLight> lights{};
    std::vector<BeaconTarget> targets{};
};

class BeaconDetector {
public:
    explicit BeaconDetector(DetectorConfig config = {});
    ~BeaconDetector() = default;

    BeaconDetector(const BeaconDetector&) = default;
    BeaconDetector& operator=(const BeaconDetector&) = default;
    BeaconDetector(BeaconDetector&&) noexcept = default;
    BeaconDetector& operator=(BeaconDetector&&) noexcept = default;

    [[nodiscard]] std::vector<BeaconTarget> detect(const cv::Mat& frame);
    [[nodiscard]] BeaconDebugResult detectDebug(const cv::Mat& frame);
    [[nodiscard]] std::optional<BeaconTarget> selectBestTarget(const std::vector<BeaconTarget>& targets) const;

    void setEnemyColor(EnemyColor color) noexcept;
    void setConfig(const DetectorConfig& config) noexcept;
    [[nodiscard]] EnemyColor enemyColor() const noexcept;
    [[nodiscard]] const DetectorConfig& config() const noexcept;

private:
    DetectorConfig config_;
};

using LightBar = BeaconLight;
using ArmorTarget = BeaconTarget;
using ArmorDetector = BeaconDetector;

}  // namespace rm_gimbal
