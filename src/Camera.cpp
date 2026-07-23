#include "Camera.hpp"

#include <DxImageProc.h>
#include <GxIAPI.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <utility>

namespace {

// 统一检查 Galaxy SDK 返回值，失败时打印具体是哪一步出错。
bool checkStatus(GX_STATUS status, const char* action) {
    if (status == GX_STATUS_SUCCESS) {
        return true;
    }

    std::cerr << "[GalaxyCamera] " << action << " failed, status = " << status << std::endl;
    return false;
}

void printLastError(const char* action) {
    GX_STATUS last_error = GX_STATUS_SUCCESS;
    char error_text[256]{};
    size_t error_text_size = sizeof(error_text);
    if (GXGetLastError(&last_error, error_text, &error_text_size) == GX_STATUS_SUCCESS) {
        std::cerr << "[GalaxyCamera] " << action << " last error: "
                  << last_error << ", " << error_text << std::endl;
    }
}

bool openDeviceByIndex(uint32_t index, GX_DEV_HANDLE* handle) {
    char index_text[16]{};
    std::snprintf(index_text, sizeof(index_text), "%u", index);

    GX_OPEN_PARAM open_param{};
    open_param.pszContent = index_text;
    open_param.openMode = GX_OPEN_INDEX;
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;

    GX_STATUS status = GXOpenDevice(&open_param, handle);
    if (status == GX_STATUS_SUCCESS) {
        return true;
    }

    std::cerr << "[GalaxyCamera] GXOpenDevice EXCLUSIVE failed, status = " << status << std::endl;
    printLastError("GXOpenDevice EXCLUSIVE");

    open_param.accessMode = GX_ACCESS_CONTROL;
    status = GXOpenDevice(&open_param, handle);
    if (status == GX_STATUS_SUCCESS) {
        std::cerr << "[GalaxyCamera] opened device with CONTROL access mode." << std::endl;
        return true;
    }

    std::cerr << "[GalaxyCamera] GXOpenDevice CONTROL failed, status = " << status << std::endl;
    printLastError("GXOpenDevice CONTROL");
    return false;
}

// 某些相机参数在不同型号或状态下可能不可写，设置前先判断能否写入。
bool isWritable(GX_DEV_HANDLE handle, GX_FEATURE_ID_CMD feature) {
    bool writable = false;
    return GXIsWritable(handle, feature, &writable) == GX_STATUS_SUCCESS && writable;
}

// 尝试设置枚举型参数；不可写时静默跳过，避免调试阶段因非关键参数失败而中断。
void trySetEnum(GX_DEV_HANDLE handle, GX_FEATURE_ID_CMD feature, int64_t value) {
    if (isWritable(handle, feature)) {
        GXSetEnum(handle, feature, value);
    }
}

// Galaxy SDK 的 Bayer 像素格式需要转换成 DxImageProc 使用的 Bayer 排列枚举。
DX_PIXEL_COLOR_FILTER bayerLayoutFromPixelFormat(int pixel_format) {
    switch (pixel_format) {
        case GX_PIXEL_FORMAT_BAYER_RG8:
            return BAYERRG;
        case GX_PIXEL_FORMAT_BAYER_GB8:
            return BAYERGB;
        case GX_PIXEL_FORMAT_BAYER_GR8:
            return BAYERGR;
        case GX_PIXEL_FORMAT_BAYER_BG8:
            return BAYERBG;
        default:
            return BAYERGB;
    }
}

const char* pixelFormatName(int pixel_format) {
    switch (pixel_format) {
        case GX_PIXEL_FORMAT_MONO8:
            return "MONO8";
        case GX_PIXEL_FORMAT_BGR8:
            return "BGR8";
        case GX_PIXEL_FORMAT_RGB8:
            return "RGB8";
        case GX_PIXEL_FORMAT_BAYER_RG8:
            return "BAYER_RG8";
        case GX_PIXEL_FORMAT_BAYER_GB8:
            return "BAYER_GB8";
        case GX_PIXEL_FORMAT_BAYER_GR8:
            return "BAYER_GR8";
        case GX_PIXEL_FORMAT_BAYER_BG8:
            return "BAYER_BG8";
        default:
            return "UNKNOWN";
    }
}

}  // namespace

Camera::Camera() = default;

Camera::~Camera() {
    release();
}

Camera::Camera(Camera&& other) noexcept
    : device_handle_(other.device_handle_),
      sdk_initialized_(other.sdk_initialized_),
      stream_on_(other.stream_on_),
      convert_buffer_(std::move(other.convert_buffer_)) {
    other.device_handle_ = nullptr;
    other.sdk_initialized_ = false;
    other.stream_on_ = false;
}

Camera& Camera::operator=(Camera&& other) noexcept {
    if (this != &other) {
        release();
        device_handle_ = other.device_handle_;
        sdk_initialized_ = other.sdk_initialized_;
        stream_on_ = other.stream_on_;
        convert_buffer_ = std::move(other.convert_buffer_);

        other.device_handle_ = nullptr;
        other.sdk_initialized_ = false;
        other.stream_on_ = false;
    }
    return *this;
}

// 打开 MER/Galaxy 相机并启动采集流。config_path 预留给后续读取曝光、增益等参数。
bool Camera::open(const std::string& config_path) {
    (void)config_path;

    if (device_handle_ != nullptr && stream_on_) {
        return true;
    }

    if (!sdk_initialized_) {
        if (!checkStatus(GXInitLib(), "GXInitLib")) {
            return false;
        }
        sdk_initialized_ = true;
    }

    uint32_t device_count = 0;
    if (!checkStatus(GXUpdateDeviceList(&device_count, 1000), "GXUpdateDeviceList")) {
        return false;
    }
    if (device_count == 0) {
        std::cerr << "[GalaxyCamera] no MER/Galaxy camera found." << std::endl;
        return false;
    }

    if (!openDeviceByIndex(1, reinterpret_cast<GX_DEV_HANDLE*>(&device_handle_))) {
        return false;
    }

    auto handle = static_cast<GX_DEV_HANDLE>(device_handle_);

    // 连续采集、关闭外触发和自动参数；调试阶段先让取流路径和画面亮度稳定。
    trySetEnum(handle, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    trySetEnum(handle, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    trySetEnum(handle, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    trySetEnum(handle, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    trySetEnum(handle, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);

    // MER-139-210U3C 是彩色相机。优先尝试 BGR8/RGB8，失败再使用 BayerRG8。
    if (isWritable(handle, GX_ENUM_PIXEL_FORMAT)) {
        GX_STATUS status = GXSetEnum(handle, GX_ENUM_PIXEL_FORMAT, GX_PIXEL_FORMAT_BGR8);
        if (status != GX_STATUS_SUCCESS) {
            status = GXSetEnum(handle, GX_ENUM_PIXEL_FORMAT, GX_PIXEL_FORMAT_RGB8);
        }
        if (status != GX_STATUS_SUCCESS) {
            GXSetEnum(handle, GX_ENUM_PIXEL_FORMAT, GX_PIXEL_FORMAT_BAYER_RG8);
        }
    }

    if (!checkStatus(GXStreamOn(handle), "GXStreamOn")) {
        GXCloseDevice(handle);
        device_handle_ = nullptr;
        return false;
    }

    stream_on_ = true;
    std::cout << "[GalaxyCamera] opened MER/Galaxy camera, device count = " << device_count << std::endl;
    return true;
}

// 从 SDK 取一帧图像，转换成 OpenCV 使用的 BGR Mat，然后把 SDK 缓冲区归还。
bool Camera::getFrame(cv::Mat& frame) {
    if (device_handle_ == nullptr || !stream_on_) {
        return false;
    }

    auto handle = static_cast<GX_DEV_HANDLE>(device_handle_);
    PGX_FRAME_BUFFER frame_buffer = nullptr;
    const GX_STATUS status = GXDQBuf(handle, &frame_buffer, 100);
    if (status == GX_STATUS_TIMEOUT) {
        return false;
    }
    if (status != GX_STATUS_SUCCESS || frame_buffer == nullptr) {
        checkStatus(status, "GXDQBuf");
        return false;
    }

    bool ok = false;
    if (frame_buffer->nStatus == GX_FRAME_STATUS_SUCCESS && frame_buffer->pImgBuf != nullptr) {
        static bool printed_frame_info = false;
        if (!printed_frame_info) {
            std::cout << "[GalaxyCamera] first frame: "
                      << frame_buffer->nWidth << "x" << frame_buffer->nHeight
                      << ", pixel format = " << pixelFormatName(frame_buffer->nPixelFormat)
                      << " (" << frame_buffer->nPixelFormat << ")" << std::endl;
            printed_frame_info = true;
        }

        ok = convertFrame(frame_buffer->pImgBuf,
                          frame_buffer->nWidth,
                          frame_buffer->nHeight,
                          frame_buffer->nPixelFormat,
                          frame);
    } else {
        std::cerr << "[GalaxyCamera] bad frame, frame status = "
                  << (frame_buffer ? frame_buffer->nStatus : -999) << std::endl;
    }

    // GXDQBuf 拿到的是 SDK 内部缓冲区，处理完必须归还，否则后续采集会卡住。
    GXQBuf(handle, frame_buffer);
    return ok && !frame.empty();
}

// 停止采集并释放 SDK 资源。析构函数也会调用，保证异常退出时尽量清理干净。
void Camera::release() {
    if (device_handle_ != nullptr) {
        auto handle = static_cast<GX_DEV_HANDLE>(device_handle_);

        if (stream_on_) {
            GXStreamOff(handle);
            stream_on_ = false;
        }

        GXCloseDevice(handle);
        device_handle_ = nullptr;
    }

    if (sdk_initialized_) {
        GXCloseLib();
        sdk_initialized_ = false;
    }
}

// 将 Galaxy SDK 的图像格式统一转换为 OpenCV BGR，后续检测模块只处理一种颜色格式。
bool Camera::convertFrame(void* image_buffer, int width, int height, int pixel_format, cv::Mat& frame) {
    if (image_buffer == nullptr || width <= 0 || height <= 0) {
        return false;
    }

    switch (pixel_format) {
        case GX_PIXEL_FORMAT_MONO8: {
            // 灰度相机或灰度输出模式：转成三通道，保持后续算法接口一致。
            cv::Mat gray(height, width, CV_8UC1, image_buffer);
            cv::cvtColor(gray, frame, cv::COLOR_GRAY2BGR);
            return true;
        }

        case GX_PIXEL_FORMAT_BGR8: {
            // SDK 已经输出 BGR，直接 clone，归还 SDK 缓冲区后 Mat 仍然有效。
            cv::Mat bgr(height, width, CV_8UC3, image_buffer);
            frame = bgr.clone();
            return true;
        }

        case GX_PIXEL_FORMAT_RGB8: {
            // SDK 给的是 RGB 顺序，OpenCV 默认显示和处理使用 BGR 顺序。
            cv::Mat rgb(height, width, CV_8UC3, image_buffer);
            cv::cvtColor(rgb, frame, cv::COLOR_RGB2BGR);
            return true;
        }

        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_BG8: {
            // 彩色工业相机常见输出是 Bayer8，需要先去马赛克得到 BGR 图像。
            const std::size_t output_size = static_cast<std::size_t>(width) *
                                            static_cast<std::size_t>(height) * 3U;
            convert_buffer_.resize(output_size);

            const auto bayer_layout = bayerLayoutFromPixelFormat(pixel_format);
            const int convert_status = DxRaw8toRGB24Ex(image_buffer,
                                                       convert_buffer_.data(),
                                                       static_cast<VxUint32>(width),
                                                       static_cast<VxUint32>(height),
                                                       RAW2RGB_NEIGHBOUR,
                                                       bayer_layout,
                                                       false,
                                                       DX_ORDER_BGR);
            if (convert_status != 0) {
                std::cerr << "[GalaxyCamera] DxRaw8toRGB24Ex failed, status = "
                          << convert_status << std::endl;
                return false;
            }

            cv::Mat bgr(height, width, CV_8UC3, convert_buffer_.data());
            frame = bgr.clone();
            return true;
        }

        default:
            std::cerr << "[GalaxyCamera] unsupported pixel format: " << pixel_format << std::endl;
            return false;
    }
}
