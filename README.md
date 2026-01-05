# 🤖 ROS2 差分驱动自主巡检机器人

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Platform](https://img.shields.io/badge/Platform-Jetson%20Nano-green)](https://developer.nvidia.com/embedded/jetson-nano)
[![MCU](https://img.shields.io/badge/MCU-STM32F103-orange)](https://www.st.com/en/microcontrollers-microprocessors/stm32f103.html)
[![License](https://img.shields.io/badge/License-Apache%202.0-red)](LICENSE)

一套完整的室内自主巡检机器人系统，包含 **ROS2 上位机导航** 和 **STM32 下位机控制**，支持 SLAM 建图、自主导航和多航点巡检任务。

---

## ✨ 功能特性

| 功能 | 说明 |
|------|------|
| 🗺️ **SLAM 建图** | slam_toolbox 实时2D激光SLAM，地图分辨率2cm |
| 🧭 **自主导航** | Nav2 路径规划与动态避障（Dijkstra + DWB） |
| 📍 **精确定位** | AMCL 自适应蒙特卡洛定位，精度 5.2cm/40m（误差率0.13%） |
| 🔄 **传感器融合** | EKF 融合编码器+IMU，Yaw精度 < 0.5° |
| 🚶 **自动巡检** | 多航点自动巡检 + 任务执行 + 日志记录 |
| 🎛️ **RQT 控制面板** | Start/Pause/Resume/Stop + 状态监控 + 初始位姿设置 |

---

## 📊 性能指标（实测）

| 指标 | 数值 | 说明 |
|------|------|------|
| 地图分辨率 | 2.0 cm/像素 | slam_toolbox 配置 |
| **导航定位精度** | **5.2 cm / 40m** | AMCL 定位（误差率0.13%，工业级） |
| 里程计闭环精度 | 2.31 cm / 32m | 误差率0.072%，AGV级别 |
| 通信错误率 | 0.00% | 编码器串口通信 |
| 最大线速度 | 5.36 cm/s | 8 RPM 电机限制 |
| 最大角速度 | 26.29°/s | 标定后轮距 0.23364m |

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    Jetson Nano (上位机)                          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────────┐ │
│  │ Nav2     │  │ SLAM     │  │ EKF      │  │ patrol_manager   │ │
│  │ 导航栈   │  │ Toolbox  │  │ 融合     │  │ 巡检管理器       │ │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └────────┬─────────┘ │
│       │             │             │                  │           │
│       └─────────────┴──────┬──────┴──────────────────┘           │
│                            │ /cmd_vel                            │
│                     ┌──────▼──────┐                              │
│                     │serial_bridge│ ──── /dev/cmd_usb ────┐      │
│                     └─────────────┘                       │      │
│  ┌─────────────┐                                          │      │
│  │ odom_pub    │ ◄──── /dev/encoder_usb ◄───────┐         │      │
│  └─────────────┘                                │         │      │
│  ┌─────────────┐                                │         ▼      │
│  │ imu_pub     │ ◄──── /dev/imu_usb             │  ┌────────────┐│
│  └─────────────┘                                │  │ RPLidar S2L││
│  ┌─────────────┐                                │  └────────────┘│
│  │ rplidar     │ ◄──── /dev/ttyUSB0             │                │
│  └─────────────┘                                │                │
└─────────────────────────────────────────────────┼────────────────┘
                                                  │
┌─────────────────────────────────────────────────┼────────────────┐
│                    STM32F103 (下位机)           │                │
│                                                 │                │
│  ┌────────────┐    ┌────────────┐    ┌─────────▼──────┐         │
│  │ USART1     │◄───│ 速度指令   │◄───│ Jetson         │         │
│  │ 接收       │    │ 解析       │    │ /dev/cmd_usb   │         │
│  └────────────┘    └─────┬──────┘    └────────────────┘         │
│                          │                                       │
│                   ┌──────▼──────┐                                │
│                   │ Modbus RTU  │                                │
│                   │ USART3      │                                │
│                   └──────┬──────┘                                │
│                          │                                       │
│         ┌────────────────┼────────────────┐                      │
│         ▼                ▼                ▼                      │
│  ┌────────────┐   ┌────────────┐   ┌────────────┐               │
│  │ 左轮电机   │   │ 右轮电机   │   │ 编码器     │───► Jetson    │
│  │ ZLAC8015D  │   │ ZLAC8015D  │   │ 50Hz反馈   │               │
│  └────────────┘   └────────────┘   └────────────┘               │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔧 硬件配置

| 组件 | 型号/规格 |
|------|-----------|
| 主控 | Jetson Nano SUPER 8GB |
| 下位机 | STM32F103C8T6 |
| 激光雷达 | 思岚 RPLidar S2L（360°扫描） |
| IMU | JY901B 10轴（航向角自动归零） |
| 编码器 | 4096 PPR（1024线×4倍频） |
| 电机驱动 | ZLAC8015D（Modbus RTU） |
| 电机 | 8 RPM 直流减速电机 |

### 机器人尺寸
- **整体尺寸**：45cm × 38cm × 15cm
- **轮径**：64.5mm（标定值）
- **轮距**：233.64mm（旋转标定）

---

## 📁 项目结构

```
ros2-patrol-robot/
├── src/robot_bringup_cpp/          # ROS2 主功能包
│   ├── src/                        # C++ 节点源码
│   │   ├── serial_bridge_node.cpp  # 速度指令桥接（逆运动学）
│   │   ├── odom_publisher_node.cpp # 编码器里程计（正向运动学）
│   │   └── imu_publisher_node.cpp  # IMU 驱动（含自动校准）
│   ├── scripts/                    # Python 脚本
│   │   ├── patrol_manager.py       # 巡检管理器
│   │   └── waypoint_recorder.py    # 航点采集工具
│   ├── config/                     # 配置文件
│   │   ├── nav2_params.yaml        # Nav2 导航参数
│   │   ├── slam_toolbox_real.yaml  # SLAM 参数
│   │   └── ekf_localization.yaml   # EKF 融合参数
│   └── launch/                     # Launch 文件
│       ├── jetson_slam_real.launch.py       # SLAM 建图
│       ├── jetson_navigation_client.launch.py # 自主导航
│       └── jetson_patrol.launch.py          # 自动巡检
├── stm32_firmware/                 # STM32 下位机固件
│   ├── User/main.c                 # 主程序
│   ├── Hardware/                   # 硬件驱动
│   │   ├── Serial.c/h              # 串口通信
│   │   ├── modbus.c/h              # Modbus RTU 协议
│   │   └── timer.c/h               # 定时器（50Hz）
│   └── Project.uvprojx             # Keil MDK 工程
├── maps/                           # 地图文件
├── test_scripts/                   # 测试脚本
├── 快速启动指南.md                 # 中文快速指南
└── 技术文档.md                     # 详细技术文档
```

---

## 🚀 快速开始

### 环境要求

- Ubuntu 22.04
- ROS2 Humble
- Jetson Nano（推荐 8GB 版本）

### 1️⃣ 编译

```bash
cd ~/ros2-patrol-robot
colcon build --symlink-install
source install/setup.bash
```

### 2️⃣ SLAM 建图

```bash
# 终端1 - 串口桥接
ros2 run robot_bringup_cpp serial_bridge_node

# 终端2 - 里程计
ros2 run robot_bringup_cpp odom_publisher_node_cpp

# 终端3 - SLAM
ros2 launch robot_bringup_cpp jetson_slam_real.launch.py

# 终端4 - 键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 3️⃣ 自主导航

```bash
# 终端1-2：同上

# 终端3 - 导航（All-in-One）
ros2 launch robot_bringup_cpp jetson_navigation_client.launch.py
```

### 4️⃣ 自动巡检

```bash
# 一键启动（包含所有节点）
ros2 launch robot_bringup_cpp jetson_patrol.launch.py
```

---

## 🏷️ 版本说明

本项目包含针对不同环境优化的版本：

| 标签 | 适用场景 | 主要特点 |
|------|---------|---------|
| `v1.0-lab-high-precision` | 实验室/办公室 | 开启scan_matching，高精度 |
| `v1.1-greenhouse-corridor` | 大棚/长廊 | 关闭scan_matching，适配狭窄走廊 |
| `v1.0-perfect-calibration` | 里程计标定 | 闭环精度0.072%，AGV级别 |

### 切换版本

```bash
# 查看所有版本
git tag -l

# 切换到实验室版本
git checkout v1.0-lab-high-precision

# 切换到大棚/长廊版本
git checkout v1.1-greenhouse-corridor

# 回到最新版本
git checkout master
```

---

## 📖 详细文档

- [快速启动指南](快速启动指南.md) - 操作手册
- [技术文档](技术文档.md) - 系统架构与参数说明
- [建图最佳实践](建图最佳实践.md) - SLAM 建图技巧
- [温室大棚巡检指南](温室大棚巡检指南.md) - 大棚环境配置

---

## 🔗 相关链接

- [ROS2 Humble 文档](https://docs.ros.org/en/humble/)
- [Nav2 文档](https://navigation.ros.org/)
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)

---

## 📄 License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

---

## 👨‍💻 作者

- **zmr0307** - [GitHub](https://github.com/zmr0307)

---

<p align="center">
  <i>如果这个项目对您有帮助，请给一个 ⭐ Star！</i>
</p>
