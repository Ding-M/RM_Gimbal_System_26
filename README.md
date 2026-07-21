# RM_Gimbal_System_26

RM_Gimbal_System_26 是一个面向 RoboMaster 云台视觉跟踪任务的工程项目。系统使用工业相机采集图像，识别红/蓝信标目标，完成目标筛选、时序稳定、PnP 位姿解算，并通过串口向下位机发送 yaw/pitch 控制指令，使云台持续跟随目标移动。

## 功能概览

- 工业相机实时采集图像，支持 Galaxy/MER 相机 SDK。
- 红蓝双方信标颜色快速切换。
- 基于颜色分割、轮廓筛选和几何约束的信标检测。
- 显示原始信标候选框、稳定目标框、二值化 mask 和控制状态。
- 目标时序稳定，降低检测框跳动对云台控制的影响。
- PnP 解算目标相对相机的 yaw、pitch 和距离。
- Tracking / Lost / Searching 控制状态机。
- 丢失目标后短时保持控制量，长时间丢失后进入搜索模式。
- 通过串口发送云台控制指令。
- 提供相机圆点板标定工具。
- 提供串口方向/协议探测工具和串口回环测试工具。

## 工程结构

├── CMakeLists.txt
├── main.cpp                         # 主程序：信标检测、调试显示、串口发送
├── config/
│   └── camera_params.yaml           # 相机参数和标定结果
├── include/
│   ├── Camera.hpp                   # 相机封装
│   ├── Detector.hpp                 # 信标检测接口
│   ├── TargetTracker.hpp            # 目标时序稳定
│   ├── Solver.hpp                   # PnP 位姿解算
│   ├── Transform.hpp                # 坐标系转换
│   ├── Controller.hpp               # 控制状态机接口
│   └── SerialHandler.hpp            # 串口协议封装
└── src/
    ├── Camera.cpp
    ├── Detector.cpp
    ├── TargetTracker.cpp
    ├── Solver.cpp
    ├── Transform.cpp
    ├── Controller.cpp
    ├── SerialHandler.cpp
    ├── Calibrator.cpp               # 相机标定工具
    ├── GimbalDirectionTest.cpp      # 云台方向/协议探测工具
    └── SerialLoopbackTest.cpp       # USB-TTL 回环测试工具


## 环境依赖

- Ubuntu / Linux
- CMake >= 3.10
- C++17 编译器
- OpenCV
- Daheng Galaxy / MER 相机 SDK
- USB-TTL 串口模块

当前 `CMakeLists.txt` 中 Galaxy SDK 默认路径为：

```cmake
set(GALAXY_SDK_ROOT "/home/mdi/YT/camera_driver")
```

如果 SDK 安装位置不同，需要修改该路径。

## 编译

```bash
cd ~/Desktop/RM_Gimbal_System_26
cmake -S . -B build
cmake --build build -j4
```

编译后会生成以下可执行文件：

```text
build/RM_Gimbal_System_26      # 主程序
build/calibrate_camera         # 相机标定工具
build/gimbal_direction_test    # 云台方向/协议测试工具
build/serial_loopback_test     # 串口回环测试工具

运行与调试
1. 运行主程序
Bash
./build/RM_Gimbal_System_26

2. 运行辅助测试工具
相机标定：
Bash
./build/calibrate_camera

3.云台方向与协议探测：
Bash
./build/gimbal_direction_test

4.串口回环测试：
Bash
./build/serial_loopback_test