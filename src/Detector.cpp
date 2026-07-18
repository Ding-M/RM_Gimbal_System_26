#include "Detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>

namespace rm_gimbal {
namespace {

// 信标是一个完整发光物体，不按装甲板“上下/左右灯条配对”处理。
// 下面的阈值只用于调试显示单个发光区域，以及把相邻发光区域合并成整体信标轮廓。
constexpr double kMinLightSide = 3.0;
constexpr double kMinBeaconSide = 10.0;
constexpr double kMinBeaconArea = 80.0;

// 将旋转矩形四个角点按左上、右上、右下、左下排序，方便后续绘制或位姿估计。
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

// 生成红/蓝信标灯块的二值图：
// 先用 BGR 通道差分突出目标颜色，再叠加高亮区域，适合识别发光信标。
cv::Mat buildColorMask(const cv::Mat& frame, const DetectorConfig& config) {
    std::vector<cv::Mat> bgr_channels;
    cv::split(frame, bgr_channels);

    // 红色/橙红信标：R 通道明显强于 B 通道；蓝色信标：B 通道明显强于 R 通道。
    cv::Mat color_diff;
    if (config.enemy_color == EnemyColor::Red) {
        cv::subtract(bgr_channels[2], bgr_channels[0], color_diff);
    } else {
        cv::subtract(bgr_channels[0], bgr_channels[2], color_diff);
    }

    cv::Mat color_mask;
    cv::threshold(color_diff, color_mask, config.binary_threshold, 255.0, cv::THRESH_BINARY);

    // 发光块通常亮度较高，用灰度阈值去掉背景里颜色相近但不发光的区域。
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    cv::Mat brightness_mask;
    cv::threshold(gray, brightness_mask, config.binary_threshold, 255.0, cv::THRESH_BINARY);

    cv::Mat mask;
    cv::bitwise_and(color_mask, brightness_mask, mask);

    // 闭运算补齐发光块内部的小孔洞，开运算去掉零散噪声。
    const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 3));
    const cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, close_kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, open_kernel);

    return mask;
}

// 将零散发光块膨胀并闭运算，得到“整个信标物体”的候选区域。
// 这样绿色框跟随的是信标整体外轮廓，而不是某两条灯块之间的组合关系。
cv::Mat buildBeaconObjectMask(const cv::Mat& light_mask) {
    cv::Mat object_mask;

    // 先横纵向适度膨胀，把同一信标上的多个发光窗口连接起来。
    const cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 9));
    cv::dilate(light_mask, object_mask, dilate_kernel, cv::Point(-1, -1), 1);

    // 再做闭运算，补齐信标内部因为亮度不均造成的小断裂。
    const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(17, 11));
    cv::morphologyEx(object_mask, object_mask, cv::MORPH_CLOSE, close_kernel);

    return object_mask;
}

// 判断调试用的单个发光块中心是否落在整体信标框内，用于估计这个候选目标包含多少有效灯块。
bool lightInsideBeacon(const BeaconLight& light, const cv::RotatedRect& beacon_rect) {
    const std::vector<cv::Point2f> points = orderPoints(beacon_rect);
    std::vector<cv::Point2f> contour{points[0], points[1], points[2], points[3]};
    return cv::pointPolygonTest(contour, light.rect.center, false) >= 0.0;
}

// 从合并后的整体 mask 中寻找信标物体。绿色框由这里输出，因此不再依赖灯条配对。
std::vector<BeaconTarget> findBeaconObjects(const cv::Mat& light_mask,
                                            const std::vector<BeaconLight>& lights,
                                            const DetectorConfig& config) {
    const cv::Mat object_mask = buildBeaconObjectMask(light_mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(object_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<BeaconTarget> targets;
    targets.reserve(contours.size());

    for (const auto& contour : contours) {
        if (contour.size() < 4) {
            continue;
        }

        const double contour_area = cv::contourArea(contour);
        if (contour_area < std::max(kMinBeaconArea, config.min_light_area)) {
            continue;
        }

        const cv::RotatedRect rect = cv::minAreaRect(contour);
        const double width = std::max<double>(rect.size.width, 1.0);
        const double height = std::max<double>(rect.size.height, 1.0);
        if (width < kMinBeaconSide || height < kMinBeaconSide) {
            continue;
        }

        // 信标外框接近 50mm x 67mm，真实拍摄允许透视变形，所以这里只做较宽松的整体比例过滤。
        const double aspect_ratio = std::max(width, height) / std::min(width, height);
        if (aspect_ratio < config.min_light_aspect_ratio || aspect_ratio > config.max_light_aspect_ratio) {
            continue;
        }

        const double rect_area = width * height;
        const double rectangularity = contour_area / rect_area;
        if (rectangularity < config.min_rectangularity) {
            continue;
        }

        std::vector<BeaconLight> object_lights;
        for (const auto& light : lights) {
            if (lightInsideBeacon(light, rect)) {
                object_lights.push_back(light);
            }
        }
        if (static_cast<int>(object_lights.size()) < config.min_lights_per_beacon) {
            continue;
        }

        BeaconTarget target;
        target.bounding_rect = rect;
        target.image_points = orderPoints(rect);
        target.lights = object_lights;
        target.center = rect.center;
        target.label = "beacon_object";

        // 置信度更偏向“整体像不像一个稳定信标物体”，灯块数量只作为辅助条件。
        const double light_count_score = std::min(1.0, static_cast<double>(object_lights.size()) / 6.0);
        const double rectangularity_score = std::clamp(rectangularity, 0.0, 1.0);
        const double area_score = std::min(1.0, contour_area / 3000.0);
        target.confidence = 0.50 * rectangularity_score + 0.30 * area_score + 0.20 * light_count_score;

        targets.push_back(target);
    }

    std::sort(targets.begin(), targets.end(), [](const BeaconTarget& lhs, const BeaconTarget& rhs) {
        return lhs.confidence > rhs.confidence;
    });

    return targets;
}

// 从二值图中提取单个信标灯块：
// 信标窗口是发光矩形，所以重点看面积、宽高比、轮廓填充度，而不再要求灯条竖直成对。
std::vector<BeaconLight> findBeaconLights(const cv::Mat& mask, const DetectorConfig& config) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<BeaconLight> lights;
    lights.reserve(contours.size());

    for (const auto& contour : contours) {
        if (contour.size() < 4) {
            continue;
        }

        const double area = cv::contourArea(contour);
        if (area < config.min_light_area) {
            continue;
        }

        const cv::RotatedRect rect = cv::minAreaRect(contour);
        const double width = std::max<double>(rect.size.width, 1.0);
        const double height = std::max<double>(rect.size.height, 1.0);
        if (width < kMinLightSide || height < kMinLightSide) {
            continue;
        }

        // 信标灯块可能横向或纵向摆放，因此使用 max/min 的宽高比描述矩形形状。
        const double aspect_ratio = std::max(width, height) / std::min(width, height);
        if (aspect_ratio < config.min_light_aspect_ratio || aspect_ratio > config.max_light_aspect_ratio) {
            continue;
        }

        // rectangularity 越接近 1，说明轮廓越像填满的矩形发光块。
        const double rect_area = width * height;
        const double rectangularity = area / rect_area;
        if (rectangularity < config.min_rectangularity) {
            continue;
        }

        lights.push_back(BeaconLight{rect, area, rectangularity});
    }

    std::sort(lights.begin(), lights.end(), [](const BeaconLight& lhs, const BeaconLight& rhs) {
        return lhs.rect.center.x == rhs.rect.center.x ? lhs.rect.center.y < rhs.rect.center.y
                                                      : lhs.rect.center.x < rhs.rect.center.x;
    });

    return lights;
}

}  // namespace

BeaconDetector::BeaconDetector(DetectorConfig config) : config_(std::move(config)) {}

// 检测主入口：颜色/亮度分割 -> 单个发光矩形提取 -> 多灯块聚类成信标 -> 置信度排序。
std::vector<BeaconTarget> BeaconDetector::detect(const cv::Mat& frame) {
    return detectDebug(frame).targets;
}

// 调试检测入口：额外返回 mask 和单个发光块，方便在画面上检查每一步效果。
BeaconDebugResult BeaconDetector::detectDebug(const cv::Mat& frame) {
    BeaconDebugResult result;
    if (frame.empty()) {
        return result;
    }

    result.mask = buildColorMask(frame, config_);
    result.lights = findBeaconLights(result.mask, config_);
    result.targets = findBeaconObjects(result.mask, result.lights, config_);

    return result;
}

// 从候选信标中选择置信度最高的一个。
std::optional<BeaconTarget> BeaconDetector::selectBestTarget(const std::vector<BeaconTarget>& targets) const {
    if (targets.empty()) {
        return std::nullopt;
    }

    return *std::max_element(targets.begin(), targets.end(), [](const BeaconTarget& lhs, const BeaconTarget& rhs) {
        return lhs.confidence < rhs.confidence;
    });
}

void BeaconDetector::setEnemyColor(EnemyColor color) noexcept {
    config_.enemy_color = color;
}

void BeaconDetector::setConfig(const DetectorConfig& config) noexcept {
    config_ = config;
}

EnemyColor BeaconDetector::enemyColor() const noexcept {
    return config_.enemy_color;
}

const DetectorConfig& BeaconDetector::config() const noexcept {
    return config_;
}

}  // namespace rm_gimbal
