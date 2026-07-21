#pragma once

#include <cstdint>
#include <array>
#include <optional>
#include <string>
#include <vector>

namespace rm_gimbal {

enum class AimMode : std::uint8_t {
    Idle = 0,
    AutoAim = 1,
    SmallBuff = 2,
    BigBuff = 3
};

struct SerialConfig {
    std::string port{"/dev/ttyUSB1"};
    int baud_rate{115200};
    int read_timeout_ms{10};
};

struct GimbalCommand {
    bool control{false};
    std::uint8_t mode_override{0};
    float yaw_deg{0.0F};
    float yaw_vel{0.0F};
    float yaw_acc{0.0F};
    float pitch_deg{0.0F};
    float pitch_vel{0.0F};
    float pitch_acc{0.0F};
    float distance_m{0.0F};
    bool fire{false};
};

struct RefereeState {
    AimMode mode{AimMode::Idle};
    std::array<float, 4> q{1.0F, 0.0F, 0.0F, 0.0F};
    float yaw{0.0F};
    float yaw_vel{0.0F};
    float pitch{0.0F};
    float pitch_vel{0.0F};
    float bullet_speed_mps{0.0F};
    std::uint16_t bullet_count{0};
};

class SerialHandler {
public:
    explicit SerialHandler(SerialConfig config = {});
    ~SerialHandler();

    SerialHandler(const SerialHandler&) = delete;
    SerialHandler& operator=(const SerialHandler&) = delete;
    SerialHandler(SerialHandler&& other) noexcept;
    SerialHandler& operator=(SerialHandler&& other) noexcept;

    bool open();
    void close();
    [[nodiscard]] bool isOpened() const noexcept;

    bool sendCommand(const GimbalCommand& command);
    [[nodiscard]] std::optional<RefereeState> readState();

    [[nodiscard]] const SerialConfig& config() const noexcept;

private:
    [[nodiscard]] std::vector<std::uint8_t> packCommand(const GimbalCommand& command) const;
    [[nodiscard]] std::optional<RefereeState> unpackState(const std::uint8_t* data, std::size_t size) const;

    SerialConfig config_;
    int fd_{-1};
    std::vector<std::uint8_t> rx_buffer_;
};

}  // namespace rm_gimbal
