# RM_Gimbal_System_26

RM_Gimbal_System_26 是一个面向 RoboMaster 云台视觉跟踪任务的 C++ 工程。系统通过 Daheng Galaxy / MER 工业相机采集图像，识别红/蓝信标目标，完成目标筛选、时序稳定、PnP 位姿解算和坐标转换，并通过串口向下位机发送 yaw/pitch 控制指令，使云台持续跟随目标移动。

## 功能概览

- 工业相机实时采集图像，支持 Galaxy/MER 相机 SDK。
- 支持红色、蓝色信标检测，并可在运行时快速切换。
- 基于颜色分割、轮廓筛选和几何约束提取信标灯块。
- 显示原始候选框、稳定目标框、二值化 mask、PnP 位姿和控制状态。
- 使用目标时序稳定模块降低检测框跳动对控制量的影响。
- 使用 PnP 解算目标相对相机的 yaw、pitch 和距离。
- 提供 Tracking / Lost / Searching 控制状态。
- 丢失目标后短时保持控制量，长时间丢失后进入小幅搜索模式。
- 通过串口发送云台控制指令，支持现场按键开启/关闭发送。
- 提供圆点板相机标定工具。
- 提供云台方向/协议探测工具和 USB-TTL 串口回环测试工具。

## 工程结构

```text
.
├── CMakeLists.txt
├── main.cpp                         # 主程序：信标检测、调试显示、控制量生成、串口发送
├── config/
│   └── camera_params.yaml           # 相机参数和标定结果
├── include/
│   ├── Camera.hpp                   # Galaxy/MER 相机封装
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
```

## 环境依赖

- Linux / Ubuntu
- CMake >= 3.10
- 支持 C++17 的编译器
- OpenCV
- Daheng Galaxy / MER 相机 SDK
- USB-TTL 串口模块

当前 `CMakeLists.txt` 中 Galaxy SDK 默认路径为：

```cmake
set(GALAXY_SDK_ROOT "/home/mdi/YT/camera_driver")
```

如果 SDK 安装位置不同，需要修改 `GALAXY_SDK_ROOT`，并确认以下文件存在：

```text
${GALAXY_SDK_ROOT}/include/daheng
${GALAXY_SDK_ROOT}/lib/x86_64/libgxiapi.so
```

## 编译

```bash
cd ~/Desktop/RM_Gimbal_System_26
cmake -S . -B build
cmake --build build -j4
```

编译后会生成：

```text
build/RM_Gimbal_System_26      # 主程序
build/calibrate_camera         # 相机标定工具
build/gimbal_direction_test    # 云台方向/协议测试工具
build/serial_loopback_test     # 串口回环测试工具
```

## 运行主程序

默认串口为 `/dev/ttyUSB0`，默认波特率为 `115200`：

```bash
./build/RM_Gimbal_System_26
```

也可以手动指定串口和波特率：

```bash
./build/RM_Gimbal_System_26 /dev/ttyUSB0 115200
```

主程序启动后会打开三个 OpenCV 窗口：

- `Beacon Debug - Frame`：原图、检测框、目标状态和控制量。
- `Beacon Debug - Mask`：颜色分割后的二值图。
- `Beacon Debug - Controls`：阈值、几何筛选参数和控制参数滑条。

### 主程序按键

| 按键 | 功能 |
| --- | --- |
| `r` | 切换为红色信标检测 |
| `b` | 切换为蓝色信标检测 |
| `c` | 在红/蓝信标检测之间切换 |
| `u` | 开启或关闭串口发送 |
| `t` | 开启或关闭固定角度测试模式 |
| `q` / `Esc` | 退出程序 |

程序启动后默认不发送串口控制量。确认视觉结果和控制量正常后，再按 `u` 打开串口发送。

## 相机标定

标定工具默认使用 `10 x 7` 对称圆点板，圆心间距为 `15mm`，标定结果会写入 `config/camera_params.yaml`：

```bash
./build/calibrate_camera
```

也可以指定圆点板参数和输出路径：

```bash
./build/calibrate_camera cols rows spacing_mm output.yaml
```

示例：

```bash
./build/calibrate_camera 10 7 15 config/camera_params.yaml
```

标定窗口按键：

| 按键 | 功能 |
| --- | --- |
| `Space` | 采集当前有效圆点板画面 |
| `c` | 使用已采集样本执行标定并保存 |
| `q` / `Esc` | 退出标定工具 |

建议从不同距离、不同姿态、画面不同位置采集 15 到 25 张以上样本。样本太少时工具会拒绝标定。

## 串口调试

### USB-TTL 回环测试

先短接 USB-TTL 的 TX 和 RX，再运行：

```bash
./build/serial_loopback_test /dev/ttyUSB1 115200
```

如果没有传参，默认使用 `/dev/ttyUSB1 115200`。测试通过说明电脑到 USB-TTL 的发送和接收链路正常。

### 云台方向/协议探测

用于在没有完整视觉链路参与时，单独验证电控串口协议和 yaw/pitch 正方向：

```bash
./build/gimbal_direction_test /dev/ttyUSB0 115200 8
```

参数含义：

- `/dev/ttyUSB0`：串口设备。
- `115200`：波特率。
- `8`：每次测试发送的角度步进，单位为度。

工具内按键：

| 按键 | 功能 |
| --- | --- |
| `1` | 协议 1：新协议，帧头 `5A A5`，帧尾 `7F FE`，无 CRC，yaw/pitch 按弧度 `rad` 发送，包长 29 字节 |
| `2` | 协议 2：旧协议，帧头 `FF`，帧尾 `0D`，带 CRC8，yaw/pitch 按弧度 `rad` 发送，包长 28 字节 |
| `3` | 协议 3：新协议，帧头 `5A A5`，帧尾 `7F FE`，无 CRC，yaw/pitch 按角度 `deg` 发送，包长 29 字节 |
| `m` | 切换 mode = 1 / 2 |
| `p` | 打印当前 +yaw 测试包的十六进制内容 |
| `d` / `a` | 发送 +yaw / -yaw |
| `w` / `s` | 发送 +pitch / -pitch |
| `x` | 发送停止/不控制 |
| `q` | 退出 |

主项目 `RM_Gimbal_System_26` 当前使用的是协议 3，即新协议、角度制、无 CRC：

```text
head: 5A A5
mode: 0 不控制，1 控制云台，2 控制云台并允许开火
payload: yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc
unit: yaw/pitch 为 deg
tail: 7F FE
packet size: 29 bytes
```

## 调试流程建议

1. 确认 Galaxy SDK 路径和相机权限正确，编译工程。
2. 运行 `calibrate_camera` 完成相机标定，生成真实内参。
3. 运行 `serial_loopback_test`，确认 USB-TTL 和系统串口权限正常。
4. 运行 `gimbal_direction_test`，确认电控协议、mode 和 yaw/pitch 正方向。
5. 运行 `RM_Gimbal_System_26`，先只看调试窗口，不打开串口。
6. 调整 `Beacon Debug - Controls` 中的阈值、面积、矩形度、长宽比和控制参数。
7. 确认检测框、PnP 结果和 `cmd_yaw/cmd_pitch` 连续稳定后，按 `u` 开启串口发送。

## 配置说明

`config/camera_params.yaml` 同时用于相机打开参数和 PnP 标定参数读取。主程序会读取：

- `camera_matrix`
- `distortion_coefficients`

如果未找到标定文件，或文件中没有 `camera_matrix`，主程序会使用临时近似内参来跑通调试界面。临时内参只能用于界面验证，正式距离、yaw 和 pitch 必须以真实标定结果为准。

当前主程序中信标物理尺寸设置为：

```cpp
solver_config.beacon_size_mm = cv::Size2f(50.0F, 67.0F);
```

如果实际目标尺寸不同，需要同步修改该参数，否则 PnP 距离和角度会产生偏差。

## 常见问题

### 无法打开相机

- 检查相机 USB3 连接、供电和驱动。
- 检查 Galaxy SDK 路径是否和 `CMakeLists.txt` 一致。
- 检查当前用户是否有访问相机设备的权限。

### 无法打开串口

- 检查设备名是否正确：`ls /dev/ttyUSB* /dev/ttyACM*`。
- 检查当前用户是否在 `dialout` 组。
- 尝试拔插 USB-TTL 后重新确认设备名。
- 主程序可显式指定串口：`./build/RM_Gimbal_System_26 /dev/ttyUSB0 115200`。

### 检测框抖动或误检较多

- 调整 `binary_threshold`、`min_light_area`、`rectangularity_%`、`max_aspect_x10`。
- 确认当前检测颜色是否正确。
- 优先让曝光和增益稳定，再调检测阈值。
- 检查目标灯条是否过曝、欠曝或被环境光干扰。

### 云台方向相反

- 先使用 `gimbal_direction_test` 单独验证 yaw/pitch 正方向。
- 主程序中当前发送 yaw 前做了取反：

```cpp
command.yaw_deg = clampCommand(static_cast<float>(-pose.yaw_deg * config.yaw_kp), config.max_cmd_deg);
```

如果电控坐标定义变化，需要同步调整该符号。

## 开发备注

- `src/Calibrator.cpp` 是独立标定工具，包含自己的 `main`，不要放入主程序源文件集合。
- `CORE_SRC` 中的文件会被链接到主程序 `RM_Gimbal_System_26`。
- `build/` 是编译产物目录，不建议提交其中的中间文件。
