#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class Camera {
public:
    Camera();
    ~Camera();

    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    Camera(Camera&& other) noexcept;
    Camera& operator=(Camera&& other) noexcept;

    bool open(const std::string& config_path); // 传入参数路径，方便后期调试
    bool getFrame(cv::Mat& frame);
    void release();

private:
    bool convertFrame(void* image_buffer, int width, int height, int pixel_format, cv::Mat& frame);

    void* device_handle_{nullptr};
    bool sdk_initialized_{false};
    bool stream_on_{false};
    std::vector<unsigned char> convert_buffer_;
};
