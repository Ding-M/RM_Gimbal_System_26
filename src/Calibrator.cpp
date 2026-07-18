#include "Camera.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

namespace {

constexpr const char* kWindowName = "Circle Grid Calibrator";
constexpr const char* kDefaultOutputPath = "config/camera_params.yaml";

struct CalibrationOptions {
    cv::Size pattern_size{10, 7};  // 圆点阵列：10 列、7 行，注意这里是圆心数量。
    float spacing_mm{15.0F};       // 相邻圆心距离，单位 mm。
    std::string output_path{kDefaultOutputPath};
};

struct DetectionResult {
    bool found{false};
    std::vector<cv::Point2f> centers{};
};

// 生成圆点板在真实世界中的 3D 坐标。标定板是平面，所以 z 恒为 0。
std::vector<cv::Point3f> buildObjectPoints(const CalibrationOptions& options) {
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(options.pattern_size.area()));

    for (int row = 0; row < options.pattern_size.height; ++row) {
        for (int col = 0; col < options.pattern_size.width; ++col) {
            points.emplace_back(col * options.spacing_mm, row * options.spacing_mm, 0.0F);
        }
    }

    return points;
}

// 配置圆点检测器。findCirclesGrid 会依赖 blob 检测结果来定位每个圆心。
cv::Ptr<cv::SimpleBlobDetector> createBlobDetector(unsigned char blob_color) {
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 5.0F;
    params.maxThreshold = 240.0F;
    params.thresholdStep = 10.0F;

    params.filterByColor = true;
    params.blobColor = blob_color;

    params.filterByArea = true;
    params.minArea = 20.0F;
    params.maxArea = 50000.0F;

    params.filterByCircularity = true;
    params.minCircularity = 0.55F;

    params.filterByInertia = true;
    params.minInertiaRatio = 0.25F;

    params.filterByConvexity = true;
    params.minConvexity = 0.65F;

    return cv::SimpleBlobDetector::create(params);
}

// 在当前相机画面中寻找 10x7 对称圆点阵列，成功时返回全部圆心像素坐标。
DetectionResult detectCircleGrid(const cv::Mat& frame, const CalibrationOptions& options) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0.0);

    std::vector<cv::Point2f> centers;
    const int flags = cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING;

    // 大多数 calib.io 圆点板是白底黑圆，先找黑色 blob；如果打印或光照导致反相，再尝试白色 blob。
    const auto black_blob_detector = createBlobDetector(0);
    if (cv::findCirclesGrid(gray, options.pattern_size, centers, flags, black_blob_detector)) {
        return DetectionResult{true, centers};
    }

    const auto white_blob_detector = createBlobDetector(255);
    if (cv::findCirclesGrid(gray, options.pattern_size, centers, flags, white_blob_detector)) {
        return DetectionResult{true, centers};
    }

    cv::Mat inverted;
    cv::bitwise_not(gray, inverted);
    if (cv::findCirclesGrid(inverted, options.pattern_size, centers, flags, black_blob_detector)) {
        return DetectionResult{true, centers};
    }

    return {};
}

// 计算平均重投影误差：误差越小，说明标定出的内参越能解释采集到的圆心位置。
double computeReprojectionError(const std::vector<std::vector<cv::Point3f>>& object_points,
                                const std::vector<std::vector<cv::Point2f>>& image_points,
                                const std::vector<cv::Mat>& rvecs,
                                const std::vector<cv::Mat>& tvecs,
                                const cv::Mat& camera_matrix,
                                const cv::Mat& distortion_coeffs) {
    double total_error_square = 0.0;
    std::size_t total_points = 0;

    for (std::size_t i = 0; i < object_points.size(); ++i) {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i], camera_matrix, distortion_coeffs, projected_points);

        const double error = cv::norm(image_points[i], projected_points, cv::NORM_L2);
        total_error_square += error * error;
        total_points += object_points[i].size();
    }

    return total_points == 0 ? 0.0 : std::sqrt(total_error_square / static_cast<double>(total_points));
}

// 将标定结果保存为 OpenCV FileStorage 可读取的 YAML，主程序会读取其中的 camera_matrix。
bool saveCalibration(const CalibrationOptions& options,
                     const cv::Size& image_size,
                     double rms_error,
                     double reprojection_error,
                     const cv::Mat& camera_matrix,
                     const cv::Mat& distortion_coeffs,
                     int sample_count) {
    const std::filesystem::path output_path(options.output_path);
    if (output_path.has_parent_path()) {
        std::filesystem::create_directories(output_path.parent_path());
    }

    cv::FileStorage fs(options.output_path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        std::cerr << "无法写入标定文件：" << options.output_path << std::endl;
        return false;
    }

    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "pattern_cols" << options.pattern_size.width;
    fs << "pattern_rows" << options.pattern_size.height;
    fs << "spacing_mm" << options.spacing_mm;
    fs << "sample_count" << sample_count;
    fs << "rms_error" << rms_error;
    fs << "mean_reprojection_error" << reprojection_error;
    fs << "camera_matrix" << camera_matrix;
    fs << "distortion_coefficients" << distortion_coeffs;
    fs.release();

    std::cout << "标定结果已保存到：" << options.output_path << std::endl;
    return true;
}

// 使用采集到的多组 2D 圆心和对应的 3D 圆点坐标执行相机标定。
bool calibrateAndSave(const CalibrationOptions& options,
                      const cv::Size& image_size,
                      const std::vector<std::vector<cv::Point2f>>& image_points) {
    if (image_points.size() < 10) {
        std::cerr << "样本太少，建议至少采集 15~25 张。当前：" << image_points.size() << std::endl;
        return false;
    }

    const std::vector<cv::Point3f> single_object_points = buildObjectPoints(options);
    std::vector<std::vector<cv::Point3f>> object_points(image_points.size(), single_object_points);

    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distortion_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    // flags 先保持默认模型：估计 fx/fy/cx/cy 和 k1/k2/p1/p2/k3。
    const int flags = 0;
    const double rms_error = cv::calibrateCamera(object_points,
                                                 image_points,
                                                 image_size,
                                                 camera_matrix,
                                                 distortion_coeffs,
                                                 rvecs,
                                                 tvecs,
                                                 flags);
    const double reprojection_error = computeReprojectionError(object_points,
                                                               image_points,
                                                               rvecs,
                                                               tvecs,
                                                               camera_matrix,
                                                               distortion_coeffs);

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "RMS error: " << rms_error << std::endl;
    std::cout << "Mean reprojection error: " << reprojection_error << " px" << std::endl;
    std::cout << "Camera matrix:\n" << camera_matrix << std::endl;
    std::cout << "Distortion coefficients:\n" << distortion_coeffs << std::endl;

    return saveCalibration(options,
                           image_size,
                           rms_error,
                           reprojection_error,
                           camera_matrix,
                           distortion_coeffs,
                           static_cast<int>(image_points.size()));
}

// 在标定预览窗口左上角绘制当前检测状态和按键提示。
void drawStatus(cv::Mat& view,
                const CalibrationOptions& options,
                bool found,
                int sample_count) {
    std::ostringstream line1;
    line1 << "Circle grid: " << options.pattern_size.width << "x" << options.pattern_size.height
          << " spacing=" << options.spacing_mm << "mm"
          << " samples=" << sample_count;

    const std::string line2 = "Space: capture | c: calibrate/save | q/Esc: quit";
    const std::string line3 = found ? "Grid found" : "Grid not found";
    const cv::Scalar status_color = found ? cv::Scalar(80, 220, 80) : cv::Scalar(80, 80, 255);

    cv::putText(view, line1.str(), cv::Point(16, 32), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::putText(view, line1.str(), cv::Point(16, 32), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(20, 180, 255), 1, cv::LINE_AA);
    cv::putText(view, line2, cv::Point(16, 64), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::putText(view, line2, cv::Point(16, 64), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                cv::Scalar(20, 180, 255), 1, cv::LINE_AA);
    cv::putText(view, line3, cv::Point(16, 96), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
    cv::putText(view, line3, cv::Point(16, 96), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                status_color, 1, cv::LINE_AA);
}

// 支持命令行覆盖默认标定板参数：
// ./build/calibrate_camera cols rows spacing_mm output.yaml
CalibrationOptions parseArgs(int argc, char** argv) {
    CalibrationOptions options;

    if (argc >= 3) {
        options.pattern_size = cv::Size(std::stoi(argv[1]), std::stoi(argv[2]));
    }
    if (argc >= 4) {
        options.spacing_mm = std::stof(argv[3]);
    }
    if (argc >= 5) {
        options.output_path = argv[4];
    }

    return options;
}

}  // namespace

int main(int argc, char** argv) {
    const CalibrationOptions options = parseArgs(argc, argv);

    std::cout << "圆点标定工具启动。" << std::endl;
    std::cout << "默认参数：10 列 x 7 行，圆心间距 15mm。" << std::endl;
    std::cout << "也可以这样指定：./build/calibrate_camera cols rows spacing_mm output.yaml" << std::endl;

    Camera camera;
    if (!camera.open("config/camera_params.yaml")) {
        std::cerr << "无法打开 Galaxy/MER 相机。" << std::endl;
        return 1;
    }

    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);

    std::vector<std::vector<cv::Point2f>> collected_image_points;
    cv::Size image_size;
    cv::Mat frame;
    int failed_frame_count = 0;

    while (true) {
        // 实时取图，检测圆点阵列，并在画面上叠加圆心位置。
        if (!camera.getFrame(frame)) {
            ++failed_frame_count;
            cv::Mat waiting_view(480, 800, CV_8UC3, cv::Scalar(30, 30, 30));
            cv::putText(waiting_view, "Waiting for Galaxy camera frame...",
                        cv::Point(40, 210), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                        cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            cv::putText(waiting_view, "Check USB3 cable, camera permission, exposure, and SDK driver.",
                        cv::Point(40, 260), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                        cv::Scalar(120, 220, 255), 2, cv::LINE_AA);
            cv::putText(waiting_view, "Press q/Esc to quit.",
                        cv::Point(40, 310), cv::FONT_HERSHEY_SIMPLEX, 0.65,
                        cv::Scalar(120, 220, 255), 2, cv::LINE_AA);
            cv::imshow(kWindowName, waiting_view);

            if (failed_frame_count % 50 == 0) {
                std::cerr << "仍未获取到相机图像帧，请检查相机连接和 Galaxy SDK 权限。" << std::endl;
            }

            const int key = cv::waitKey(20);
            if (key == 'q' || key == 27) {
                break;
            }
            continue;
        }
        failed_frame_count = 0;

        image_size = frame.size();
        const DetectionResult detection = detectCircleGrid(frame, options);

        cv::Mat view = frame.clone();
        if (detection.found) {
            cv::drawChessboardCorners(view, options.pattern_size, detection.centers, true);
        }
        drawStatus(view, options, detection.found, static_cast<int>(collected_image_points.size()));
        cv::imshow(kWindowName, view);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) {
            break;
        }
        if (key == ' ' && detection.found) {
            // 只保存成功识别到完整圆点阵列的帧，避免坏样本影响标定。
            collected_image_points.push_back(detection.centers);
            std::cout << "采集第 " << collected_image_points.size() << " 张标定图。" << std::endl;
        } else if (key == 'c' || key == 'C') {
            // 使用当前已采集样本执行标定，并写入 config/camera_params.yaml。
            calibrateAndSave(options, image_size, collected_image_points);
        }
    }

    camera.release();
    cv::destroyAllWindows();
    return 0;
}
