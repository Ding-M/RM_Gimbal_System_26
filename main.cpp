#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <opencv2/opencv.hpp>
#include "Camera.hpp"
#include "Controller.hpp"
#include "Detector.hpp"
#include "Solver.hpp"
#include "TargetTracker.hpp"
#include "Transform.hpp"

namespace {

constexpr const char* kFrameWindow = "Beacon Debug - Frame";
constexpr const char* kMaskWindow = "Beacon Debug - Mask";
constexpr const char* kControlWindow = "Beacon Debug - Controls";

// 调试窗口滑条状态。OpenCV trackbar 只支持 int，所以百分比和 x10 参数在这里做缩放。
struct TrackbarState {
    int binary_threshold{80};
    int min_light_area{20};
    int min_rectangularity_percent{45};
    int max_light_aspect_ratio_x10{45};
    int min_lights_per_beacon{2};
    int yaw_kp_percent{100};
    int pitch_kp_percent{100};
    int max_cmd_deg{20};
    int search_yaw_deg{8};
};

// 云台控制策略参数。现场联调时可以用滑条调整比例系数和限幅，避免目标突然跳变导致云台猛转。
struct ControlStrategyConfig {
    double yaw_kp{1.0};
    double pitch_kp{1.0};
    float max_cmd_deg{20.0F};
    std::size_t lost_frame_threshold{5};
    std::size_t search_frame_threshold{30};
    float search_yaw_deg{8.0F};
    double fire_yaw_tolerance_deg{1.0};
    double fire_pitch_tolerance_deg{1.0};
};

// 主调试界面里的控制指令状态。这里只做显示，不发送串口，方便先验收视觉到控制量是否连续。
struct ControlDebugState {
    rm_gimbal::TrackingState state{rm_gimbal::TrackingState::Idle};
    rm_gimbal::GimbalCommand command{};
    rm_gimbal::GimbalCommand last_command{};
    std::size_t lost_frames{0};
};

// 固定角度测试模式。用于把视觉链路和串口链路拆开验证：先发一个绝对 yaw/pitch，看云台是否响应。
struct FixedTargetState {
    bool enabled{false};
    float yaw_deg{30.0F};
    float pitch_deg{0.0F};
};

// 串口发送状态。主程序启动后默认不发送，现场按 u 后才打开串口并发送控制量。
struct SerialDebugState {
    std::string port{"/dev/ttyUSB0"};
    int baud_rate{115200};
    bool enabled{false};
    bool opened{false};
    bool last_send_ok{false};
    std::string message{"serial off"};
};

struct ResponseDebugState {
    int yaw_response_ms{-1};
    int pitch_response_ms{-1};
    bool yaw_active{false};
    bool pitch_active{false};
};

std::string trackingStateName(rm_gimbal::TrackingState state);

std::string timestampForFile() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    const std::tm local_time = *std::localtime(&now_time);

    std::ostringstream text;
    text << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return text.str();
}

std::string responseText(int response_ms) {
    if (response_ms < 0) {
        return "--";
    }
    return std::to_string(response_ms) + "ms";
}

class ResponseAxisTracker {
public:
    ResponseAxisTracker(double start_threshold_deg,
                        double stable_threshold_deg,
                        int stable_frame_threshold)
        : start_threshold_deg_(start_threshold_deg),
          stable_threshold_deg_(stable_threshold_deg),
          stable_frame_threshold_(stable_frame_threshold) {}

    void update(const std::optional<double>& error_deg,
                std::chrono::steady_clock::time_point now) {
        if (!error_deg.has_value()) {
            active_ = false;
            stable_frames_ = 0;
            return;
        }

        const double abs_error = std::abs(*error_deg);
        if (!active_ && abs_error >= start_threshold_deg_) {
            active_ = true;
            stable_frames_ = 0;
            start_time_ = now;
        }

        if (!active_) {
            return;
        }

        if (abs_error <= stable_threshold_deg_) {
            ++stable_frames_;
        } else {
            stable_frames_ = 0;
        }

        if (stable_frames_ >= stable_frame_threshold_) {
            last_response_ms_ = static_cast<int>(
                std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count());
            active_ = false;
            stable_frames_ = 0;
        }
    }

    [[nodiscard]] int lastResponseMs() const noexcept {
        return last_response_ms_;
    }

    [[nodiscard]] bool active() const noexcept {
        return active_;
    }

private:
    double start_threshold_deg_{5.0};
    double stable_threshold_deg_{1.0};
    int stable_frame_threshold_{10};
    bool active_{false};
    int stable_frames_{0};
    int last_response_ms_{-1};
    std::chrono::steady_clock::time_point start_time_{};
};

class ResponseTracker {
public:
    ResponseDebugState update(const std::optional<rm_gimbal::PoseResult>& pose,
                              std::chrono::steady_clock::time_point now) {
        const std::optional<double> yaw_error = pose.has_value()
                                                    ? std::optional<double>(pose->yaw_deg)
                                                    : std::nullopt;
        const std::optional<double> pitch_error = pose.has_value()
                                                      ? std::optional<double>(pose->pitch_deg)
                                                      : std::nullopt;

        yaw_.update(yaw_error, now);
        pitch_.update(pitch_error, now);

        ResponseDebugState state;
        state.yaw_response_ms = yaw_.lastResponseMs();
        state.pitch_response_ms = pitch_.lastResponseMs();
        state.yaw_active = yaw_.active();
        state.pitch_active = pitch_.active();
        return state;
    }

private:
    ResponseAxisTracker yaw_{5.0, 1.0, 10};
    ResponseAxisTracker pitch_{5.0, 1.0, 10};
};

class TelemetryLogger {
public:
    TelemetryLogger() {
        namespace fs = std::filesystem;
        fs::create_directories("logs");
        path_ = "logs/telemetry_" + timestampForFile() + ".csv";
        file_.open(path_);
        if (!file_.is_open()) {
            std::cerr << "[Telemetry] 无法创建日志文件：" << path_ << std::endl;
            return;
        }

        file_ << "time_ms,target_yaw_deg,target_pitch_deg,distance_m,"
              << "cmd_yaw_deg,cmd_pitch_deg,state,control,fire,"
              << "serial_on,serial_opened,send_ok,yaw_response_ms,pitch_response_ms\n";
        std::cout << "[Telemetry] 日志记录到：" << path_ << std::endl;
    }

    void write(std::chrono::milliseconds elapsed,
               const std::optional<rm_gimbal::PoseResult>& pose,
               const ControlDebugState& control,
               const SerialDebugState& serial,
               const ResponseDebugState& response) {
        if (!file_.is_open()) {
            return;
        }

        file_ << elapsed.count() << ',';
        if (pose.has_value()) {
            file_ << pose->yaw_deg << ','
                  << pose->pitch_deg << ','
                  << pose->distance_m << ',';
        } else {
            file_ << ",,,";
        }

        file_ << control.command.yaw_deg << ','
              << control.command.pitch_deg << ','
              << trackingStateName(control.state) << ','
              << (control.command.control ? 1 : 0) << ','
              << (control.command.fire ? 1 : 0) << ','
              << (serial.enabled ? 1 : 0) << ','
              << (serial.opened ? 1 : 0) << ','
              << (serial.last_send_ok ? 1 : 0) << ','
              << response.yaw_response_ms << ','
              << response.pitch_response_ms << '\n';
    }

    [[nodiscard]] const std::string& path() const noexcept {
        return path_;
    }

private:
    std::string path_{};
    std::ofstream file_{};
};

// 画旋转矩形，分别用于显示单个灯块和最终信标外框。
void drawRotatedRect(cv::Mat& image,
                     const cv::RotatedRect& rect,
                     const cv::Scalar& color,
                     int thickness) {
    cv::Point2f points[4];
    rect.points(points);

    for (int i = 0; i < 4; ++i) {
        cv::line(image, points[i], points[(i + 1) % 4], color, thickness, cv::LINE_AA);
    }
}

// 当前检测颜色显示给调试界面使用。
std::string colorName(rm_gimbal::EnemyColor color) {
    return color == rm_gimbal::EnemyColor::Red ? "Red" : "Blue";
}

std::string trackingStateName(rm_gimbal::TrackingState state) {
    switch (state) {
        case rm_gimbal::TrackingState::Tracking:
            return "Tracking";
        case rm_gimbal::TrackingState::Lost:
            return "Lost";
        case rm_gimbal::TrackingState::Searching:
            return "Searching";
        case rm_gimbal::TrackingState::Idle:
        default:
            return "Idle";
    }
}

// 将滑条整数值转换成 DetectorConfig，保证现场调参不用重新编译。
rm_gimbal::DetectorConfig makeDetectorConfig(const TrackbarState& state,
                                             rm_gimbal::EnemyColor enemy_color) {
    rm_gimbal::DetectorConfig config;
    config.enemy_color = enemy_color;
    config.binary_threshold = std::clamp(state.binary_threshold, 0, 255);
    config.min_light_area = std::max(1, state.min_light_area);
    config.min_light_aspect_ratio = 0.25;
    config.max_light_aspect_ratio = std::max(1, state.max_light_aspect_ratio_x10) / 10.0;
    config.min_rectangularity = std::clamp(state.min_rectangularity_percent, 0, 100) / 100.0;
    config.min_lights_per_beacon = std::max(1, state.min_lights_per_beacon);
    return config;
}

// 创建 OpenCV 调参窗口，现场可以直接拖动阈值、面积和几何筛选参数。
void createControlWindow(TrackbarState& state) {
    cv::namedWindow(kControlWindow, cv::WINDOW_NORMAL);
    cv::resizeWindow(kControlWindow, 520, 260);

    cv::createTrackbar("binary_threshold", kControlWindow, &state.binary_threshold, 255);
    cv::createTrackbar("min_light_area", kControlWindow, &state.min_light_area, 2000);
    cv::createTrackbar("rectangularity_%", kControlWindow, &state.min_rectangularity_percent, 100);
    cv::createTrackbar("max_aspect_x10", kControlWindow, &state.max_light_aspect_ratio_x10, 100);
    cv::createTrackbar("min_lights", kControlWindow, &state.min_lights_per_beacon, 10);
    cv::createTrackbar("yaw_kp_%", kControlWindow, &state.yaw_kp_percent, 300);
    cv::createTrackbar("pitch_kp_%", kControlWindow, &state.pitch_kp_percent, 300);
    cv::createTrackbar("max_cmd_deg", kControlWindow, &state.max_cmd_deg, 60);
    cv::createTrackbar("search_yaw_deg", kControlWindow, &state.search_yaw_deg, 30);
}

// 把当前检测参数格式化成一行文字，叠加显示在主画面左上角。
std::string configText(const rm_gimbal::DetectorConfig& config) {
    std::ostringstream text;
    text.precision(2);
    text << std::fixed
         << "thr=" << config.binary_threshold
         << " area=" << config.min_light_area
         << " rect=" << config.min_rectangularity
         << " aspect<" << config.max_light_aspect_ratio
         << " min_lights=" << config.min_lights_per_beacon;
    return text.str();
}

// 将滑条值转换为云台控制参数。比例系数控制跟随力度，限幅负责保护云台动作幅度。
ControlStrategyConfig makeControlConfig(const TrackbarState& state) {
    ControlStrategyConfig config;
    config.yaw_kp = std::max(0, state.yaw_kp_percent) / 100.0;
    config.pitch_kp = std::max(0, state.pitch_kp_percent) / 100.0;
    config.max_cmd_deg = static_cast<float>(std::max(1, state.max_cmd_deg));
    config.search_yaw_deg = static_cast<float>(std::max(1, state.search_yaw_deg));
    return config;
}

float clampCommand(float value, float limit) {
    return std::clamp(value, -limit, limit);
}

// 根据稳定后的 PnP 结果生成 yaw/pitch 控制量。串口开启后，这个结构体会被打包发送给下位机。
// 当前下位机 yaw 正方向与相机解算方向相反，所以发送前把 yaw 取反。
rm_gimbal::GimbalCommand makeDebugCommand(const rm_gimbal::PoseResult& pose,
                                          const ControlStrategyConfig& config) {
    rm_gimbal::GimbalCommand command;
    command.control = true;
    command.yaw_deg = clampCommand(static_cast<float>(-pose.yaw_deg * config.yaw_kp), config.max_cmd_deg);
    command.pitch_deg = clampCommand(static_cast<float>(pose.pitch_deg * config.pitch_kp), config.max_cmd_deg);
    command.distance_m = static_cast<float>(pose.distance_m);

    // 目标接近中心时置 fire=true，这里表示“已基本对准”，不代表一定真实开火。
    command.fire = std::abs(pose.yaw_deg) <= config.fire_yaw_tolerance_deg &&
                   std::abs(pose.pitch_deg) <= config.fire_pitch_tolerance_deg;
    return command;
}

rm_gimbal::GimbalCommand makeFixedTargetCommand(const FixedTargetState& fixed_target) {
    rm_gimbal::GimbalCommand command;
    command.control = true;
    command.mode_override = 2;
    command.yaw_deg = fixed_target.yaw_deg;
    command.pitch_deg = fixed_target.pitch_deg;
    return command;
}

// 简化版控制状态机：Tracking 正常输出；Lost 保留并衰减上一帧；Searching 输出小幅 yaw 扫描。
void updateControlDebug(ControlDebugState& control,
                        const std::optional<rm_gimbal::PoseResult>& pose,
                        const ControlStrategyConfig& config,
                        const FixedTargetState& fixed_target) {
    if (fixed_target.enabled) {
        control.state = rm_gimbal::TrackingState::Tracking;
        control.lost_frames = 0;
        control.command = makeFixedTargetCommand(fixed_target);
        control.command.fire = false;
        control.last_command = control.command;
        return;
    }

    if (pose.has_value()) {
        control.state = rm_gimbal::TrackingState::Tracking;
        control.lost_frames = 0;
        control.command = makeDebugCommand(*pose, config);
        control.last_command = control.command;
        return;
    }

    ++control.lost_frames;
    if (control.lost_frames <= config.lost_frame_threshold) {
        control.state = rm_gimbal::TrackingState::Lost;
        control.command = control.last_command;
        control.command.yaw_deg *= 0.8F;
        control.command.pitch_deg *= 0.8F;
        control.command.fire = false;
        control.last_command = control.command;
        return;
    }

    control.state = rm_gimbal::TrackingState::Searching;
    const float direction = (control.lost_frames / config.search_frame_threshold) % 2 == 0 ? 1.0F : -1.0F;
    control.command = {};
    control.command.control = true;
    control.command.yaw_deg = direction * config.search_yaw_deg;
    control.command.fire = false;

    if (control.lost_frames > config.search_frame_threshold * 2) {
        control.lost_frames = config.lost_frame_threshold + 1;
    }
}

rm_gimbal::EnemyColor toggleEnemyColor(rm_gimbal::EnemyColor color) {
    return color == rm_gimbal::EnemyColor::Red ? rm_gimbal::EnemyColor::Blue
                                               : rm_gimbal::EnemyColor::Red;
}

// 读取相机标定文件。标定工具会输出 camera_matrix 和 distortion_coefficients。
rm_gimbal::CameraCalibration loadCalibration(const std::string& path) {
    rm_gimbal::CameraCalibration calibration;

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "未找到相机标定文件：" << path << "，将使用临时近似内参。" << std::endl;
        return calibration;
    }

    fs["camera_matrix"] >> calibration.camera_matrix;
    fs["distortion_coefficients"] >> calibration.distortion_coeffs;
    if (calibration.distortion_coeffs.empty()) {
        fs["distortion_coeffs"] >> calibration.distortion_coeffs;
    }

    if (calibration.camera_matrix.empty()) {
        std::cerr << "标定文件中没有 camera_matrix，将使用临时近似内参。" << std::endl;
    }

    return calibration;
}

// 临时近似内参只用于没有标定文件时跑通界面；正式 PnP 必须使用真实标定结果。
rm_gimbal::CameraCalibration makeApproxCalibration(const cv::Size& image_size) {
    rm_gimbal::CameraCalibration calibration;

    // 临时内参：只用于先跑通 PnP 显示。最终距离/yaw/pitch 必须以后续棋盘格标定结果为准。
    const double focal_px = 5000.0;
    calibration.camera_matrix = (cv::Mat_<double>(3, 3) << focal_px, 0.0, image_size.width * 0.5,
                                  0.0, focal_px, image_size.height * 0.5,
                                  0.0, 0.0, 1.0);
    calibration.distortion_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    return calibration;
}

// 生成主调试画面：原图 + 灯块框 + 原始信标框 + 稳定信标框 + 参数 + PnP 位姿。
cv::Mat makeDebugView(const cv::Mat& frame,
                      const rm_gimbal::BeaconDebugResult& debug,
                      rm_gimbal::EnemyColor enemy_color,
                      const rm_gimbal::DetectorConfig& config,
                      const std::optional<rm_gimbal::BeaconTarget>& stable_target,
                      int lost_frames,
                      const std::optional<rm_gimbal::PoseResult>& pose,
                      const std::optional<rm_gimbal::CoordinateResult>& coordinates,
                      const ControlDebugState& control,
                      const SerialDebugState& serial_state,
                      const ControlStrategyConfig& control_config,
                      const FixedTargetState& fixed_target,
                      const ResponseDebugState& response_state) {
    cv::Mat view = frame.clone();

    // 黄色框：当前识别到的单个信标发光矩形块。
    for (const auto& light : debug.lights) {
        drawRotatedRect(view, light.rect, cv::Scalar(0, 255, 255), 2);
    }

    // 绿色细框：当前帧直接检测出的整体信标候选，用来观察检测原始抖动。
    for (const auto& target : debug.targets) {
        drawRotatedRect(view, target.bounding_rect, cv::Scalar(0, 180, 0), 2);

        std::ostringstream text;
        text.precision(2);
        text << std::fixed << "raw conf=" << target.confidence << " lights=" << target.lights.size();

        const cv::Point text_pos(static_cast<int>(target.center.x + 8), static_cast<int>(target.center.y - 8));
        cv::putText(view, text.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(0, 180, 0), 2, cv::LINE_AA);
    }

    // 青色粗框：经过置信度门限、低通滤波和丢失保持后的稳定目标，PnP 使用这个框。
    if (stable_target.has_value()) {
        drawRotatedRect(view, stable_target->bounding_rect, cv::Scalar(255, 255, 0), 3);
        cv::circle(view, stable_target->center, 5, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);

        std::ostringstream text;
        text.precision(2);
        text << std::fixed << "stable conf=" << stable_target->confidence
             << " lost=" << lost_frames;

        const cv::Point text_pos(static_cast<int>(stable_target->center.x + 8),
                                 static_cast<int>(stable_target->center.y + 22));
        cv::putText(view, text.str(), text_pos, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    }

    const std::string status = "Target: " + colorName(enemy_color) +
                               " | lights: " + std::to_string(debug.lights.size()) +
                               " | beacons: " + std::to_string(debug.targets.size()) +
                               " | keys: c/r/b color, u serial, q quit";
    cv::putText(view, status, cv::Point(16, 32), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::putText(view, status, cv::Point(16, 32), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(20, 180, 255), 1, cv::LINE_AA);

    const std::string params = configText(config);
    cv::putText(view, params, cv::Point(16, 62), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::putText(view, params, cv::Point(16, 62), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(80, 220, 80), 1, cv::LINE_AA);

    if (pose.has_value()) {
        std::ostringstream pose_text;
        pose_text.precision(2);
        pose_text << std::fixed
                  << "distance=" << pose->distance_m << "m"
                  << " yaw=" << pose->yaw_deg << "deg"
                  << " pitch=" << pose->pitch_deg << "deg";
        cv::putText(view, pose_text.str(), cv::Point(16, 92), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
        cv::putText(view, pose_text.str(), cv::Point(16, 92), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(80, 180, 255), 1, cv::LINE_AA);
    }

    {
        std::ostringstream control_text;
        control_text.precision(2);
        control_text << std::fixed
                     << "state=" << trackingStateName(control.state)
                     << " cmd_yaw=" << control.command.yaw_deg << "deg"
                     << " cmd_pitch=" << control.command.pitch_deg << "deg"
                     << " control=" << (control.command.control ? 1 : 0)
                     << " fire=" << (control.command.fire ? 1 : 0);
        cv::putText(view, control_text.str(), cv::Point(16, 122), cv::FONT_HERSHEY_SIMPLEX, 0.58,
                    cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
        cv::putText(view, control_text.str(), cv::Point(16, 122), cv::FONT_HERSHEY_SIMPLEX, 0.58,
                    cv::Scalar(255, 170, 80), 1, cv::LINE_AA);
    }

    {
        std::ostringstream serial_text;
        serial_text << "serial=" << (serial_state.enabled ? "ON" : "OFF")
                    << " opened=" << (serial_state.opened ? 1 : 0)
                    << " send=" << (serial_state.last_send_ok ? "OK" : "--")
                    << " port=" << serial_state.port
                    << " | key: u toggle";
        cv::putText(view, serial_text.str(), cv::Point(16, 152), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
        cv::putText(view, serial_text.str(), cv::Point(16, 152), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(120, 220, 255), 1, cv::LINE_AA);
    }

    {
        std::ostringstream strategy_text;
        strategy_text.precision(2);
        strategy_text << std::fixed
                      << "kp yaw/pitch=" << control_config.yaw_kp << "/" << control_config.pitch_kp
                      << " max_cmd=" << control_config.max_cmd_deg << "deg"
                      << " search=" << control_config.search_yaw_deg << "deg"
                      << " fixed=" << (fixed_target.enabled ? "ON" : "OFF")
                      << " t=" << fixed_target.yaw_deg << "deg/" << fixed_target.pitch_deg << "deg";
        cv::putText(view, strategy_text.str(), cv::Point(16, 182), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
        cv::putText(view, strategy_text.str(), cv::Point(16, 182), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(180, 180, 255), 1, cv::LINE_AA);
    }

    {
        std::ostringstream response_line;
        response_line << "response yaw=" << responseText(response_state.yaw_response_ms)
                      << (response_state.yaw_active ? "*" : "")
                      << " pitch=" << responseText(response_state.pitch_response_ms)
                      << (response_state.pitch_active ? "*" : "")
                      << " | key: p screenshot";
        cv::putText(view, response_line.str(), cv::Point(16, 212), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
        cv::putText(view, response_line.str(), cv::Point(16, 212), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(120, 255, 180), 1, cv::LINE_AA);
    }

    if (coordinates.has_value()) {
        std::ostringstream camera_text;
        std::ostringstream gimbal_text;
        std::ostringstream world_text;
        camera_text.precision(1);
        gimbal_text.precision(1);
        world_text.precision(1);

        camera_text << std::fixed << "camera(mm): x=" << coordinates->camera_mm[0]
                    << " y=" << coordinates->camera_mm[1]
                    << " z=" << coordinates->camera_mm[2];
        gimbal_text << std::fixed << "gimbal(mm): x=" << coordinates->gimbal_mm[0]
                    << " y=" << coordinates->gimbal_mm[1]
                    << " z=" << coordinates->gimbal_mm[2];
        world_text << std::fixed << "world(mm):  x=" << coordinates->world_mm[0]
                   << " y=" << coordinates->world_mm[1]
                   << " z=" << coordinates->world_mm[2];

        const std::array<std::string, 3> lines{camera_text.str(), gimbal_text.str(), world_text.str()};
        for (std::size_t i = 0; i < lines.size(); ++i) {
            const cv::Point pos(16, static_cast<int>(242 + i * 28));
            cv::putText(view, lines[i], pos, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
            cv::putText(view, lines[i], pos, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        cv::Scalar(180, 220, 80), 1, cv::LINE_AA);
        }
    }

    return view;
}

}  // namespace

int main(int argc, char** argv) {
    // 打开 Galaxy/MER 工业相机，后续所有图像都来自外接相机。
    Camera cam;
    if (!cam.open("config/camera_params.yaml")) {
        std::cerr << "无法打开摄像头！" << std::endl;
        return -1;
    }

    // 检测器负责输出信标像素位置；解算器负责把像素位置变成相机坐标系下的位姿。
    rm_gimbal::BeaconDetector detector;
    rm_gimbal::SolverConfig solver_config;
    solver_config.beacon_size_mm = cv::Size2f(50.0F, 67.0F);
    solver_config.calibration = loadCalibration("config/camera_params.yaml");
    rm_gimbal::BallisticSolver solver(solver_config);
    bool using_approx_calibration = solver_config.calibration.camera_matrix.empty();
    rm_gimbal::CoordinateTransformer transformer;
    rm_gimbal::GimbalState gimbal_state;
    rm_gimbal::TargetTracker tracker;
    ControlDebugState control_debug;
    FixedTargetState fixed_target;

    SerialDebugState serial_debug;
    if (argc >= 2) {
        serial_debug.port = argv[1];
    }
    if (argc >= 3) {
        serial_debug.baud_rate = std::stoi(argv[2]);
    }
    rm_gimbal::SerialConfig serial_config;
    serial_config.port = serial_debug.port;
    serial_config.baud_rate = serial_debug.baud_rate;
    rm_gimbal::SerialHandler serial(serial_config);

    rm_gimbal::EnemyColor enemy_color = rm_gimbal::EnemyColor::Blue;
    TrackbarState trackbars;
    createControlWindow(trackbars);
    ResponseTracker response_tracker;
    TelemetryLogger telemetry_logger;
    const auto telemetry_start_time = std::chrono::steady_clock::now();

    cv::Mat frame;
    std::cout << "进入信标检测调试界面：按 r 检测红色，按 b 检测蓝色，按 u 开关串口发送，按 p 保存截图，按 q 或 Esc 退出。"
              << std::endl;
    std::cout << "串口默认：" << serial_debug.port << " @ " << serial_debug.baud_rate
              << "，也可使用 ./build/RM_Gimbal_System_26 /dev/ttyUSB0 115200 指定。" << std::endl;

    while (true) {
        // 1. 取图。
        if (!cam.getFrame(frame)) break;

        // 2. 读取滑条参数并更新检测器。
        const auto config = makeDetectorConfig(trackbars, enemy_color);
        const auto control_config = makeControlConfig(trackbars);
        detector.setConfig(config);

        // 3. 执行信标检测，同时拿到 mask、单灯块和最终信标目标。
        const auto debug = detector.detectDebug(frame);

        if (using_approx_calibration) {
            solver.setCalibration(makeApproxCalibration(frame.size()));
            using_approx_calibration = false;
            std::cerr << "已启用临时近似内参，仅用于调试显示；请尽快完成相机标定。" << std::endl;
        }

        // 4. 选取置信度最高的信标，再做时序稳定，最后把稳定目标送入 PnP 位姿解算。
        std::optional<rm_gimbal::PoseResult> pose;
        std::optional<rm_gimbal::CoordinateResult> coordinates;
        const auto best_target = detector.selectBestTarget(debug.targets);
        const auto stable_target = tracker.update(best_target);
        if (stable_target.has_value()) {
            pose = solver.solve(*stable_target);
            if (pose.has_value()) {
                coordinates = transformer.cameraToWorld(pose->translation_vector, gimbal_state);
            }
        }

        // 5. 控制状态机：Tracking 跟踪，Lost 短时保持，Searching 丢失后小幅扫描。
        updateControlDebug(control_debug, pose, control_config, fixed_target);

        // 6. 串口发送只在按 u 开启后执行。发送内容就是当前画面显示的 cmd_yaw/cmd_pitch。
        serial_debug.opened = serial.isOpened();
        serial_debug.last_send_ok = false;
        if (serial_debug.enabled && serial.isOpened()) {
            serial_debug.last_send_ok = serial.sendCommand(control_debug.command);
            serial_debug.message = serial_debug.last_send_ok ? "send ok" : "send failed";
        }

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - telemetry_start_time);
        const ResponseDebugState response_state = response_tracker.update(pose, now);
        telemetry_logger.write(elapsed, pose, control_debug, serial_debug, response_state);

        // 7. 显示调试画面和二值化 mask。
        const cv::Mat debug_view = makeDebugView(frame, debug, enemy_color, config,
                                                 stable_target, tracker.lostFrames(),
                                                 pose, coordinates, control_debug, serial_debug,
                                                 control_config, fixed_target, response_state);

        cv::imshow(kFrameWindow, debug_view);
        cv::imshow(kMaskWindow, debug.mask);

        // 8. 键盘控制：红蓝切换、串口发送开关、固定测试模式和退出。
        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
        if (key == 'c' || key == 'C') {
            enemy_color = toggleEnemyColor(enemy_color);
            tracker.reset();
            control_debug = {};
            std::cout << "切换为" << (enemy_color == rm_gimbal::EnemyColor::Red ? "红色" : "蓝色")
                      << "信标检测。" << std::endl;
        } else if (key == 'r' || key == 'R') {
            enemy_color = rm_gimbal::EnemyColor::Red;
            tracker.reset();
            control_debug = {};
            std::cout << "切换为红色信标检测。" << std::endl;
        } else if (key == 'b' || key == 'B') {
            enemy_color = rm_gimbal::EnemyColor::Blue;
            tracker.reset();
            control_debug = {};
            std::cout << "切换为蓝色信标检测。" << std::endl;
        } else if (key == 'u' || key == 'U') {
            if (!serial_debug.enabled) {
                serial_debug.enabled = serial.open();
                serial_debug.opened = serial.isOpened();
                serial_debug.message = serial_debug.enabled ? "serial enabled" : "open failed";
                std::cout << "串口发送：" << (serial_debug.enabled ? "开启 " : "开启失败 ")
                          << serial_debug.port << " @ " << serial_debug.baud_rate << std::endl;
            } else {
                rm_gimbal::GimbalCommand stop_command;
                serial.sendCommand(stop_command);
                serial.close();
                serial_debug.enabled = false;
                serial_debug.opened = false;
                serial_debug.last_send_ok = false;
                serial_debug.message = "serial off";
                std::cout << "串口发送：关闭。" << std::endl;
            }
        } else if (key == 't' || key == 'T') {
            fixed_target.enabled = !fixed_target.enabled;
            control_debug = {};
            std::cout << "固定角度测试模式：" << (fixed_target.enabled ? "开启" : "关闭")
                      << "，yaw=" << fixed_target.yaw_deg
                      << " deg, pitch=" << fixed_target.pitch_deg << " deg" << std::endl;
        } else if (key == 'p' || key == 'P') {
            std::filesystem::create_directories("logs");
            const std::string screenshot_path = "logs/screenshot_" + timestampForFile() + ".png";
            if (cv::imwrite(screenshot_path, debug_view)) {
                std::cout << "调试截图已保存：" << screenshot_path << std::endl;
            } else {
                std::cerr << "调试截图保存失败：" << screenshot_path << std::endl;
            }
        }
    }

    if (serial.isOpened()) {
        rm_gimbal::GimbalCommand stop_command;
        serial.sendCommand(stop_command);
        serial.close();
    }
    cam.release();
    cv::destroyAllWindows();
    std::cout << "程序安全退出。" << std::endl;
    return 0;
}
