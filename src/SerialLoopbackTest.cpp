#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

namespace {

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
    tty.c_cc[VTIME] = 5;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "tcsetattr failed: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

void printHex(const std::vector<std::uint8_t>& data) {
    std::cout << std::hex << std::uppercase << std::setfill('0');
    for (const auto byte : data) {
        std::cout << std::setw(2) << static_cast<int>(byte) << ' ';
    }
    std::cout << std::dec << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
    std::string port = "/dev/ttyUSB1";
    int baud_rate = 115200;
    if (argc >= 2) {
        port = argv[1];
    }
    if (argc >= 3) {
        baud_rate = std::stoi(argv[2]);
    }

    const int fd = openSerial(port, baud_rate);
    if (fd < 0) {
        return 1;
    }

    const std::vector<std::uint8_t> tx{0x5A, 0xA5, 0x01, 0x11, 0x22, 0x33, 0x7F, 0xFE};
    std::cout << "请先短接 USB-TTL 的 TX 和 RX，再按回车开始回环测试。" << std::endl;
    std::cin.get();

    std::cout << "TX: ";
    printHex(tx);
    const ssize_t written = ::write(fd, tx.data(), tx.size());
    if (written != static_cast<ssize_t>(tx.size())) {
        std::cerr << "写入失败: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::array<std::uint8_t, 256> rx{};
    const ssize_t bytes = ::read(fd, rx.data(), rx.size());
    if (bytes <= 0) {
        std::cerr << "没有读到回环数据。USB-TTL、端口或权限可能有问题。" << std::endl;
        ::close(fd);
        return 1;
    }

    std::vector<std::uint8_t> received(rx.begin(), rx.begin() + bytes);
    std::cout << "RX: ";
    printHex(received);

    if (received == tx) {
        std::cout << "回环测试通过：电脑和 USB-TTL 串口发送/接收正常。" << std::endl;
    } else {
        std::cout << "读到了数据，但和发送内容不一致，请检查线或串口参数。" << std::endl;
    }

    ::close(fd);
    return 0;
}
