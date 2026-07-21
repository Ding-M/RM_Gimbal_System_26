#include "Controller.hpp"

#include <opencv2/highgui.hpp>

#include <algorithm>
#include <chrono>
#include <thread>
#include <utility>

namespace rm_gimbal {

GimbalController::GimbalController(Camera camera,
                                   ArmorDetector detector,
                                   BallisticSolver solver,
                                   SerialHandler serial,
                                   ControllerConfig config)
    : camera_(std::move(camera)),
      detector_(std::move(detector)),
      solver_(std::move(solver)),
      serial_(std::move(serial)),
      config_(config) {}

GimbalController::~GimbalController() {
    stop();
}

// 初始化整套控制链路：相机必须打开；串口是否打开由配置决定，方便离线调试。
bool GimbalController::initialize() {
    if (!camera_.open("config/camera_params.yaml")) {
        state_ = TrackingState::Idle;
        return false;
    }

    if (config_.enable_serial && !serial_.isOpened()) {
        if (!serial_.open()) {
            state_ = TrackingState::Idle;
            return false;
        }
    }

    state_ = TrackingState::Idle;
    lost_frames_ = 0;
    last_command_ = {};
    return true;
}

// 连续处理每一帧。这个函数不负责复杂 UI，只负责稳定地产生控制指令。
void GimbalController::run() {
    running_ = true;

    while (running_) {
        const auto result = processOnce();
        if (!result.has_value()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        if (config_.enable_debug_view && cv::waitKey(1) == 27) {
            stop();
        }
    }
}

void GimbalController::stop() noexcept {
    running_ = false;
    camera_.release();
    if (serial_.isOpened()) {
        serial_.close();
    }
}

bool GimbalController::isRunning() const noexcept {
    return running_;
}

// 单帧控制流程：
// 1. 取图
// 2. 检测信标
// 3. 选择最优目标并 PnP 解算
// 4. 更新状态机
// 5. 生成云台命令并按需通过串口发送
std::optional<FrameResult> GimbalController::processOnce() {
    cv::Mat frame;
    if (!camera_.getFrame(frame)) {
        updateState(false);
        return std::nullopt;
    }

    FrameResult result;
    const auto targets = detector_.detect(frame);
    result.target = detector_.selectBestTarget(targets);

    if (result.target.has_value()) {
        result.pose = solver_.solve(*result.target);
    }

    const bool target_found = result.pose.has_value();
    updateState(target_found);

    if (target_found) {
        result.command = makeCommand(*result.pose);
        last_command_ = result.command;
    } else if (state_ == TrackingState::Lost) {
        // 短暂丢失目标时，不立刻清零，保留上一帧方向并衰减，避免云台抖动。
        result.command = last_command_;
        result.command.yaw_deg *= 0.8F;
        result.command.pitch_deg *= 0.8F;
        result.command.fire = false;
        last_command_ = result.command;
    } else if (state_ == TrackingState::Searching) {
        // 长时间丢失目标后进入搜索模式，输出一个小幅 yaw 扫描命令。
        const float direction = (lost_frames_ / config_.search_frame_threshold) % 2 == 0 ? 1.0F : -1.0F;
        result.command.control = true;
        result.command.yaw_deg = direction * config_.search_yaw_deg;
        result.command.pitch_deg = 0.0F;
        result.command.distance_m = 0.0F;
        result.command.fire = false;
        last_command_ = result.command;
    } else {
        result.command = {};
        last_command_ = {};
    }

    if (config_.enable_serial && serial_.isOpened()) {
        result.command_sent = serial_.sendCommand(result.command);
    }

    result.state = state_;
    result.lost_frames = lost_frames_;
    return result;
}

const ControllerConfig& GimbalController::config() const noexcept {
    return config_;
}

TrackingState GimbalController::state() const noexcept {
    return state_;
}

// 根据 PnP 结果生成云台控制指令。
// 这里先用比例控制：目标相对相机的 yaw/pitch 误差越大，输出角度命令越大。
GimbalCommand GimbalController::makeCommand(const PoseResult& pose) const {
    GimbalCommand command;
    command.control = true;
    command.yaw_deg = std::clamp(static_cast<float>(-1.0*pose.yaw_deg * config_.yaw_kp),
                                 -config_.max_command_deg,
                                 config_.max_command_deg);
    command.pitch_deg = std::clamp(static_cast<float>(pose.pitch_deg * config_.pitch_kp),
                                   -config_.max_command_deg,
                                   config_.max_command_deg);
    command.distance_m = static_cast<float>(pose.distance_m);
    command.fire = shouldFire(pose);
    return command;
}

// 目标已经接近画面中心时才允许 fire。桌面小云台无发射机构时，这个字段也可作为“已对准”标志。
bool GimbalController::shouldFire(const PoseResult& pose) const noexcept {
    return std::abs(pose.yaw_deg) <= config_.fire_yaw_tolerance_deg &&
           std::abs(pose.pitch_deg) <= config_.fire_pitch_tolerance_deg;
}

// 控制状态机：
// Idle      无目标或刚启动
// Tracking  正常跟踪目标
// Lost      短暂丢失，保留上一帧控制量并衰减
// Searching 长时间丢失，进入左右扫描搜索
void GimbalController::updateState(bool target_found) noexcept {
    if (target_found) {
        state_ = TrackingState::Tracking;
        lost_frames_ = 0;
        return;
    }

    ++lost_frames_;
    if (lost_frames_ <= config_.lost_frame_threshold) {
        state_ = TrackingState::Lost;
    } else if (lost_frames_ <= config_.search_frame_threshold) {
        state_ = TrackingState::Searching;
    } else {
        // 防止 lost_frames_ 无限增大，同时保持搜索方向周期稳定。
        lost_frames_ = config_.lost_frame_threshold + 1;
        state_ = TrackingState::Searching;
    }
}

}  // namespace rm_gimbal
