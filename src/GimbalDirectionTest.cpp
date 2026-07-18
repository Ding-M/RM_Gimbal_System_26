#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
constexpr std::uint8_t kCrc8Init = 0xff;
constexpr std::array<std::uint8_t, 256> kCrc8Table{
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

enum class ProtocolVariant {
    NewRad = 1,
    OldRad = 2,
    NewDeg = 3,
};

struct TestCommand {
    bool control{false};
    bool fire{false};
    std::uint8_t mode_override{0};
    float yaw_deg{0.0F};
    float pitch_deg{0.0F};
};

struct __attribute__((packed)) NewVisionToGimbal {
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

struct __attribute__((packed)) OldVisionToGimbal {
    std::uint8_t head{0xFF};
    std::uint8_t mode{0};
    float yaw{0.0F};
    float yaw_vel{0.0F};
    float yaw_acc{0.0F};
    float pitch{0.0F};
    float pitch_vel{0.0F};
    float pitch_acc{0.0F};
    std::uint8_t crc8{0};
    std::uint8_t tail{0x0D};
};

static_assert(sizeof(NewVisionToGimbal) == 29);
static_assert(sizeof(OldVisionToGimbal) == 28);

std::uint8_t crc8(const std::uint8_t* data, std::uint16_t len) {
    std::uint8_t crc = kCrc8Init;
    while (len--) {
        crc = kCrc8Table[crc ^ *data++];
    }
    return crc;
}

float degToRad(float deg) {
    return static_cast<float>(static_cast<double>(deg) * kDegToRad);
}

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

int openSerial(const std::string& port, int baud_rate) {
    const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "无法打开串口 " << port << ": " << std::strerror(errno) << std::endl;
        return -1;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "tcgetattr failed: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    const speed_t baud = baudRateToTermios(baud_rate);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

bool writeAll(int fd, const std::vector<std::uint8_t>& packet) {
    std::size_t total = 0;
    while (total < packet.size()) {
        const ssize_t written = ::write(fd, packet.data() + total, packet.size() - total);
        if (written < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "write failed: " << std::strerror(errno) << std::endl;
            return false;
        }
        total += static_cast<std::size_t>(written);
    }
    return true;
}

template <typename Packet>
std::vector<std::uint8_t> toBytes(const Packet& packet) {
    const auto* begin = reinterpret_cast<const std::uint8_t*>(&packet);
    return {begin, begin + sizeof(Packet)};
}

std::vector<std::uint8_t> packNew(const TestCommand& command, bool radians) {
    NewVisionToGimbal packet;
    packet.mode = command.control ? (command.mode_override != 0 ? command.mode_override : (command.fire ? 2U : 1U)) : 0U;
    packet.yaw = command.control ? (radians ? degToRad(command.yaw_deg) : command.yaw_deg) : 0.0F;
    packet.pitch = command.control ? (radians ? degToRad(command.pitch_deg) : command.pitch_deg) : 0.0F;
    return toBytes(packet);
}

std::vector<std::uint8_t> packOld(const TestCommand& command) {
    OldVisionToGimbal packet;
    packet.mode = command.control ? (command.mode_override != 0 ? command.mode_override : (command.fire ? 2U : 1U)) : 0U;
    packet.yaw = command.control ? degToRad(command.yaw_deg) : 0.0F;
    packet.pitch = command.control ? degToRad(command.pitch_deg) : 0.0F;
    packet.crc8 = crc8(reinterpret_cast<const std::uint8_t*>(&packet), sizeof(packet) - 2);
    return toBytes(packet);
}

std::vector<std::uint8_t> packCommand(const TestCommand& command, ProtocolVariant variant) {
    switch (variant) {
        case ProtocolVariant::NewRad:
            return packNew(command, true);
        case ProtocolVariant::OldRad:
            return packOld(command);
        case ProtocolVariant::NewDeg:
            return packNew(command, false);
    }
    return packNew(command, true);
}

std::string hexDump(const std::vector<std::uint8_t>& packet) {
    std::ostringstream out;
    out << std::hex << std::uppercase << std::setfill('0');
    for (const auto byte : packet) {
        out << std::setw(2) << static_cast<int>(byte) << ' ';
    }
    return out.str();
}

void readBack(int fd) {
    std::array<std::uint8_t, 256> buffer{};
    const ssize_t bytes = ::read(fd, buffer.data(), buffer.size());
    if (bytes <= 0) {
        std::cout << "RX: 无回传字节" << std::endl;
        return;
    }

    std::vector<std::uint8_t> data(buffer.begin(), buffer.begin() + bytes);
    std::cout << "RX (" << bytes << " bytes): " << hexDump(data) << std::endl;
}

const char* variantName(ProtocolVariant variant) {
    switch (variant) {
        case ProtocolVariant::NewRad:
            return "1: GIMBAL v2.0 5A A5 / 7F FE, yaw/pitch=rad, no CRC";
        case ProtocolVariant::OldRad:
            return "2: OLD FF / 0D, yaw/pitch=rad, CRC8";
        case ProtocolVariant::NewDeg:
            return "3: NEW 5A A5 / 7F FE, yaw/pitch=deg, no CRC";
    }
    return "unknown";
}

void printHelp(float step_deg, ProtocolVariant variant, std::uint8_t mode) {
    std::cout << "\n云台方向/协议探测工具\n"
              << "当前协议: " << variantName(variant) << "\n"
              << "当前 mode: " << static_cast<int>(mode) << "\n"
              << "当前步进角度: " << step_deg << " deg\n"
              << "按键:\n"
              << "  1/2/3: 切换协议候选\n"
              << "  m: 切换 mode=1/2\n"
              << "  p: 打印当前 +yaw 测试包十六进制\n"
              << "  d/a: +yaw / -yaw\n"
              << "  w/s: +pitch / -pitch\n"
              << "  x: 停止/不控制\n"
              << "  q: 退出\n"
              << "建议每种协议都试 d/w。如果某种协议云台会动，就记录协议编号。\n"
              << std::endl;
}

TestCommand makeCommand(float yaw_deg, float pitch_deg, std::uint8_t mode) {
    TestCommand command;
    command.control = true;
    command.mode_override = mode;
    command.yaw_deg = yaw_deg;
    command.pitch_deg = pitch_deg;
    return command;
}

void sendForDuration(int fd,
                     const TestCommand& command,
                     ProtocolVariant variant,
                     std::chrono::milliseconds duration) {
    const auto packet = packCommand(command, variant);
    const auto end_time = std::chrono::steady_clock::now() + duration;

    // 连续发帧，兼容需要周期控制帧的电控。
    while (std::chrono::steady_clock::now() < end_time) {
        writeAll(fd, packet);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    writeAll(fd, packCommand({}, variant));
    readBack(fd);
}

}  // namespace

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB0";
    int baud_rate = 115200;
    if (argc >= 2) {
        port = argv[1];
    }
    if (argc >= 3) {
        baud_rate = std::stoi(argv[2]);
    }

    const float step_deg = argc >= 4 ? std::stof(argv[3]) : 8.0F;
    int fd = openSerial(port, baud_rate);
    if (fd < 0) {
        return 1;
    }

    ProtocolVariant variant = ProtocolVariant::NewRad;
    std::uint8_t mode = 1;
    std::cout << "已打开串口: " << port << ", baud=" << baud_rate << std::endl;
    printHelp(step_deg, variant, mode);

    char key = '\0';
    while (std::cin >> key) {
        TestCommand command;
        bool should_send = true;

        switch (key) {
            case '1':
                variant = ProtocolVariant::NewRad;
                should_send = false;
                printHelp(step_deg, variant, mode);
                break;
            case '2':
                variant = ProtocolVariant::OldRad;
                should_send = false;
                printHelp(step_deg, variant, mode);
                break;
            case '3':
                variant = ProtocolVariant::NewDeg;
                should_send = false;
                printHelp(step_deg, variant, mode);
                break;
            case 'm':
                mode = mode == 1 ? 2 : 1;
                should_send = false;
                printHelp(step_deg, variant, mode);
                break;
            case 'p':
                should_send = false;
                std::cout << "TX sample: " << hexDump(packCommand(makeCommand(step_deg, 0.0F, mode), variant))
                          << std::endl;
                break;
            case 'd':
                command = makeCommand(step_deg, 0.0F, mode);
                std::cout << "发送 +yaw, " << variantName(variant) << std::endl;
                break;
            case 'a':
                command = makeCommand(-step_deg, 0.0F, mode);
                std::cout << "发送 -yaw, " << variantName(variant) << std::endl;
                break;
            case 'w':
                command = makeCommand(0.0F, step_deg, mode);
                std::cout << "发送 +pitch, " << variantName(variant) << std::endl;
                break;
            case 's':
                command = makeCommand(0.0F, -step_deg, mode);
                std::cout << "发送 -pitch, " << variantName(variant) << std::endl;
                break;
            case 'x':
                command = {};
                std::cout << "发送停止/不控制" << std::endl;
                break;
            case 'q':
                writeAll(fd, packCommand({}, variant));
                ::close(fd);
                return 0;
            default:
                should_send = false;
                printHelp(step_deg, variant, mode);
                break;
        }

        if (should_send) {
            sendForDuration(fd, command, variant, std::chrono::milliseconds(1000));
        }
    }

    writeAll(fd, packCommand({}, variant));
    ::close(fd);
    return 0;
}
