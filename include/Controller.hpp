#pragma once

#include "Camera.hpp"
#include "Detector.hpp"
#include "SerialHandler.hpp"
#include "Solver.hpp"

#include <atomic>
#include <cstddef>
#include <optional>

namespace rm_gimbal {

enum class TrackingState {
    Idle,
    Tracking,
    Lost,
    Searching
};

struct ControllerConfig {
    bool enable_serial{true};
    bool enable_debug_view{false};
    double fire_yaw_tolerance_deg{1.0};
    double fire_pitch_tolerance_deg{1.0};
    double yaw_kp{1.0};
    double pitch_kp{1.0};
    std::size_t lost_frame_threshold{5};
    std::size_t search_frame_threshold{30};
    float search_yaw_deg{8.0F};
};

struct FrameResult {
    std::optional<ArmorTarget> target{};
    std::optional<PoseResult> pose{};
    GimbalCommand command{};
    bool command_sent{false};
    TrackingState state{TrackingState::Idle};
    std::size_t lost_frames{0};
};

class GimbalController {
public:
    GimbalController(Camera camera,
                     ArmorDetector detector,
                     BallisticSolver solver,
                     SerialHandler serial,
                     ControllerConfig config = {});
    ~GimbalController();

    GimbalController(const GimbalController&) = delete;
    GimbalController& operator=(const GimbalController&) = delete;
    GimbalController(GimbalController&&) noexcept = default;
    GimbalController& operator=(GimbalController&&) noexcept = default;

    bool initialize();
    void run();
    void stop() noexcept;
    [[nodiscard]] bool isRunning() const noexcept;

    [[nodiscard]] std::optional<FrameResult> processOnce();
    [[nodiscard]] const ControllerConfig& config() const noexcept;
    [[nodiscard]] TrackingState state() const noexcept;

private:
    [[nodiscard]] GimbalCommand makeCommand(const PoseResult& pose) const;
    [[nodiscard]] bool shouldFire(const PoseResult& pose) const noexcept;
    void updateState(bool target_found) noexcept;

    Camera camera_;
    ArmorDetector detector_;
    BallisticSolver solver_;
    SerialHandler serial_;
    ControllerConfig config_;
    std::atomic_bool running_{false};
    TrackingState state_{TrackingState::Idle};
    std::size_t lost_frames_{0};
    GimbalCommand last_command_{};
};

}  // namespace rm_gimbal
