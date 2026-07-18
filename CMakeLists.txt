cmake_minimum_required(VERSION 3.10)
project(RM_Gimbal_System_26 VERSION 1.0)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 自动寻找 OpenCV 库（如果你的电脑上已经配置了 OpenCV）
find_package(OpenCV REQUIRED)

set(GALAXY_SDK_ROOT "/home/mdi/YT/camera_driver")
set(GALAXY_INCLUDE_DIR "${GALAXY_SDK_ROOT}/include/daheng")
set(GALAXY_LIBRARY "${GALAXY_SDK_ROOT}/lib/x86_64/libgxiapi.so")

# 包含头文件目录
include_directories(include ${GALAXY_INCLUDE_DIR})

# 核心源文件。Calibrator.cpp 是独立标定工具，不能放进主程序，否则会出现多个 main。
set(CORE_SRC
    src/Camera.cpp
    src/Detector.cpp
    src/TargetTracker.cpp
    src/Solver.cpp
    src/Transform.cpp
    src/Controller.cpp
    src/SerialHandler.cpp
)

# 生成可执行程序
add_executable(RM_Gimbal_System_26 ${CORE_SRC} main.cpp)
add_executable(calibrate_camera src/Camera.cpp src/Calibrator.cpp)
add_executable(gimbal_direction_test src/SerialHandler.cpp src/GimbalDirectionTest.cpp)
add_executable(serial_loopback_test src/SerialLoopbackTest.cpp)

# 链接 OpenCV 库
target_link_libraries(RM_Gimbal_System_26 ${OpenCV_LIBS} ${GALAXY_LIBRARY})
target_link_libraries(calibrate_camera ${OpenCV_LIBS} ${GALAXY_LIBRARY})
target_link_libraries(gimbal_direction_test ${OpenCV_LIBS})
target_link_libraries(serial_loopback_test ${OpenCV_LIBS})

set_target_properties(RM_Gimbal_System_26 calibrate_camera gimbal_direction_test serial_loopback_test PROPERTIES
    BUILD_RPATH "${GALAXY_SDK_ROOT}/lib/x86_64"
)
