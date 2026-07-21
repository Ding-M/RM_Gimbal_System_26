#include "SerialHandler.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <utility>

namespace rm_gimbal {
namespace {

constexpr std::array<std::uint8_t, 2> kFrameHead{0x5A, 0xA5};

// 电控给出的云台 -> 视觉协议。
// head: 5A A5, tail: 7F FE。packed 保证内存布局和串口字节流一致。
struct __attribute__((packed)) GimbalToVisionPacket {
    std::uint8_t head[2]{0x5A, 0xA5};
    std::uint8_t mode{0};
    float q[4]{1.0F, 0.0F, 0.0F, 0.0F};
    float yaw{0.0F};
    float yaw_vel{0.0F};
    float pitch{0.0F};
    float pitch_vel{0.0F};
    float bullet_speed{0.0F};
    std::uint16_t bullet_count{0};
    std::uint8_t tail[2]{0x7F, 0xFE};
};

// 视觉 -> 电控协议。
// mode 约定：0 不控制，1 控制云台，2 控制云台并允许开火。
struct __attribute__((packed)) VisionToGimbalPacket {
    std::uint8_t head[2]{0x5A, 0xA5};
    std::uint8_t mode{0};
    float yaw{0.0F};
    float yaw_vel{0.0F};
    float yaw_acc{0.0F};
    float pitch{0.0F};
    float pitch_vel{0.0F};
    float pitch_acc{0.0F};
    std::uint8_t tail[2]{0x7F, 0xFE};
};

static_assert(sizeof(GimbalToVisionPacket) == 43);
static_assert(sizeof(VisionToGimbalPacket) == 29);

speed_t baudRateToTermios(int baud_rate) {
    switch (baud_rate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        default:
            return B115200;
    }
}

bool hasValidHeader(const std::uint8_t* data) {
    return data[0] == 0x5A && data[1] == 0xA5;
}

bool hasValidTail(const std::uint8_t* data, std::size_t size) {
    return size >= 2 && data[size - 2] == 0x7F && data[size - 1] == 0xFE;
}

std::string resolveSerialPort(const std::string& configured_port) {
    namespace fs = std::filesystem;

    if (!configured_port.empty() && fs::exists(configured_port)) {
        return configured_port;
    }

    const std::array<std::string, 5> preferred_ports{
        "/dev/gimbal",
        "/dev/ttyUSB1",
        "/dev/ttyUSB0",
        "/dev/ttyACM0",
        "/dev/ttyACM1",
    };

    for (const auto& port : preferred_ports) {
        if (fs::exists(port)) {
            return port;
        }
    }

    return configured_port;
}

}  // namespace

SerialHandler::SerialHandler(SerialConfig config) : config_(std::move(config)) {}

SerialHandler::~SerialHandler() {
    close();
}

SerialHandler::SerialHandler(SerialHandler&& other) noexcept
    : config_(std::move(other.config_)), fd_(other.fd_), rx_buffer_(std::move(other.rx_buffer_)) {
    other.fd_ = -1;
}

SerialHandler& SerialHandler::operator=(SerialHandler&& other) noexcept {
    if (this != &other) {
        close();
        config_ = std::move(other.config_);
        fd_ = other.fd_;
        rx_buffer_ = std::move(other.rx_buffer_);
        other.fd_ = -1;
    }
    return *this;
}

// 打开串口并配置为 8N1 原始模式。电控协议本身已经有帧头和帧尾，所以不使用行模式。
bool SerialHandler::open() {
    if (isOpened()) {
        return true;
    }

    const std::string resolved_port = resolveSerialPort(config_.port);
    if (resolved_port != config_.port) {
        std::cerr << "[SerialHandler] configured port " << config_.port
                  << " not found, fallback to " << resolved_port << std::endl;
    }

    fd_ = ::open(resolved_port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::cerr << "[SerialHandler] open failed: " << resolved_port
                  << ", error: " << std::strerror(errno) << std::endl;
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        close();
        return false;
    }

    cfmakeraw(&tty);
    const speed_t baud = baudRateToTermios(config_.baud_rate);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = std::max(1, config_.read_timeout_ms / 100);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        std::cerr << "[SerialHandler] tcsetattr failed: " << std::strerror(errno) << std::endl;
        close();
        return false;
    }

    tcflush(fd_, TCIOFLUSH);

    return true;
}

void SerialHandler::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialHandler::isOpened() const noexcept {
    return fd_ >= 0;
}

// 发送 VisionToGimbal：
// [5A A5] [mode] [yaw/yaw_vel/yaw_acc/pitch/pitch_vel/pitch_acc(float, deg)] [7F FE]
// 电控协议约定 yaw/pitch 为角度制，Controller 内部同样使用角度制，所以这里直接发送 deg。
bool SerialHandler::sendCommand(const GimbalCommand& command) {
    if (!isOpened()) {
        return false;
    }

    const auto packet = packCommand(command);
    std::size_t total_written = 0;
    while (total_written < packet.size()) {
        const ssize_t written = ::write(fd_, packet.data() + total_written, packet.size() - total_written);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "[SerialHandler] write failed: " << std::strerror(errno) << std::endl;
            return false;
        }
        if (written == 0) {
            std::cerr << "[SerialHandler] write returned 0 bytes." << std::endl;
            return false;
        }
        total_written += static_cast<std::size_t>(written);
    }

    return true;
}

// 读取 GimbalToVision v2.0。串口是字节流，所以要滑动寻找 5A A5 帧头，再检查 7F FE 帧尾。
std::optional<RefereeState> SerialHandler::readState() {
    if (!isOpened()) {
        return std::nullopt;
    }

    std::array<std::uint8_t, 256> temp{};
    const ssize_t bytes_read = ::read(fd_, temp.data(), temp.size());
    if (bytes_read > 0) {
        rx_buffer_.insert(rx_buffer_.end(), temp.begin(), temp.begin() + bytes_read);
    }

    while (rx_buffer_.size() >= sizeof(GimbalToVisionPacket)) {
        const auto head = std::search(rx_buffer_.begin(), rx_buffer_.end(),
                                      kFrameHead.begin(), kFrameHead.end());
        if (head == rx_buffer_.end()) {
            rx_buffer_.clear();
            return std::nullopt;
        }

        rx_buffer_.erase(rx_buffer_.begin(), head);
        if (rx_buffer_.size() < sizeof(GimbalToVisionPacket)) {
            return std::nullopt;
        }

        const auto state = unpackState(rx_buffer_.data(), sizeof(GimbalToVisionPacket));
        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + sizeof(GimbalToVisionPacket));
        if (state.has_value()) {
            return state;
        }
    }

    return std::nullopt;
}

const SerialConfig& SerialHandler::config() const noexcept {
    return config_;
}

std::vector<std::uint8_t> SerialHandler::packCommand(const GimbalCommand& command) const {
    VisionToGimbalPacket packet;
    packet.mode = command.control ? (command.mode_override != 0 ? command.mode_override : (command.fire ? 2U : 1U)) : 0U;
    packet.yaw = command.control ? command.yaw_deg : 0.0F;
    packet.yaw_vel = command.yaw_vel;
    packet.yaw_acc = command.yaw_acc;
    packet.pitch = command.control ? command.pitch_deg : 0.0F;
    packet.pitch_vel = command.pitch_vel;
    packet.pitch_acc = command.pitch_acc;

    const auto* begin = reinterpret_cast<const std::uint8_t*>(&packet);
    return std::vector<std::uint8_t>(begin, begin + sizeof(packet));
}

std::optional<RefereeState> SerialHandler::unpackState(const std::uint8_t* data, std::size_t size) const {
    if (data == nullptr || size != sizeof(GimbalToVisionPacket)) {
        return std::nullopt;
    }
    if (!hasValidHeader(data) || !hasValidTail(data, size)) {
        return std::nullopt;
    }

    GimbalToVisionPacket packet;
    std::memcpy(&packet, data, sizeof(packet));

    RefereeState state;
    state.mode = static_cast<AimMode>(packet.mode);
    state.q = {packet.q[0], packet.q[1], packet.q[2], packet.q[3]};
    state.yaw = packet.yaw;
    state.yaw_vel = packet.yaw_vel;
    state.pitch = packet.pitch;
    state.pitch_vel = packet.pitch_vel;
    state.bullet_speed_mps = packet.bullet_speed;
    state.bullet_count = packet.bullet_count;
    return state;
}

}  // namespace rm_gimbal
