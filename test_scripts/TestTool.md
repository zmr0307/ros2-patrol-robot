# 测试与调试工具

本目录包含用于机器人系统测试和调试的独立脚本。

## 工具列表

### 1. check_usb_ports_v2.sh
**功能：** USB端口检查工具（开机检查 / 慢速检测可选）

**用途：**
- 检查USB设备连接状态（USB0-USB3）
- 检查设备占用情况
- 通过数据格式自动判断端口对应关系
- 验证USB端口映射是否正确

**使用方法：**
```bash
cd /home/nvidia/robot_ws
bash ./test_scripts/check_usb_ports_v2.sh          # 默认模式（约13秒/端口）
bash ./test_scripts/check_usb_ports_v2.sh --slow   # 慢速模式（约38秒/端口，推荐）
```

**或者创建别名（推荐）：**
```bash
# 添加到 ~/.bashrc
echo 'alias check_usb="cd /home/nvidia/robot_ws && bash ./test_scripts/check_usb_ports_v2.sh --slow"' >> ~/.bashrc
source ~/.bashrc

# 之后直接运行
check_usb
```

**开机自动检查（可选）：**
在 `~/.bashrc` 末尾添加：
```bash
# 开机自动检查USB端口
if [ -f /home/nvidia/robot_ws/test_scripts/check_usb_ports_v2.sh ]; then
    cd /home/nvidia/robot_ws && bash ./test_scripts/check_usb_ports_v2.sh --slow
fi
```

**说明：**
- 检查USB设备：`/dev/ttyCH341USB0` ~ `/dev/ttyCH341USB3`
- 数据格式识别：
  - **0xA5开头** → 编码器数据（STM32格式）
  - **0x55开头** → IMU数据（JY901B格式）
  - **滑动窗口识别 0x55 0x51/52/53** → IMU 数据（JY901B/JY901S 系列）
  - **滑动窗口识别多次 0xA5** → 编码器数据（STM32 固件）
  - **无持续数据** → 速度指令端口（通常空闲）
- 读取前自动临时设置串口为 115200/8N1/raw，避免乱码
- 优先检查稳定别名 `/dev/{imu,encoder,cmd}_usb`，再回退原始端口

**预期结果：**
- ✅ `/dev/cmd_usb` → serial_bridge_node (速度指令发送到STM32，对应 `/dev/ttyCH341USB0`)
- ✅ `/dev/imu_usb` → imu_publisher_node (IMU 数据接收，0x55 帧头，对应 `/dev/ttyCH341USB1`)
- ✅ `/dev/encoder_usb` → odom_publisher_node (编码器数据接收，0xA5 帧头，对应 `/dev/ttyCH341USB2`)

**典型使用场景：**
- 每次开机/换线后快速检查串口映射
- 诊断 USB 通信或别名错位问题（检测输出直接提示）
- 助于维护 udev 规则：脚本会显示当前别名指向路径

---

### 2. imu_angle_monitor.py
**功能：** IMU角度实时监控工具

**用途：**
- 实时显示IMU的 Roll、Pitch、Yaw 角度
- 验证IMU数据是否正确
- 检查机器人初始朝向

**使用方法：**
```bash
cd /home/nvidia/robot_ws
python3 test_scripts/imu_angle_monitor.py
```

**说明：**
- 订阅话题：`/imu/data_raw`
- 显示频率：10Hz
- 按 Ctrl+C 停止

**典型使用场景：**
- 建图前验证机器人是否摆正
- 测试IMU是否正常工作
- 检查IMU的安装方向

---

### 3. odom_ekf_compare.py
**功能：** 编码器里程计 vs EKF融合角度对比工具

**用途：**
- 实时对比编码器(/odom)和EKF(/odometry/filtered)的yaw角度
- 检测角度融合是否存在滞后或偏差
- 诊断旋转时激光与地图偏移的问题

**使用方法：**
```bash
cd /home/nvidia/robot_ws
python3 test_scripts/odom_ekf_compare.py
```

**说明：**
- 订阅话题：`/odom`, `/odometry/filtered`
- 显示内容：编码器角度、EKF角度、两者差值
- 状态指示：
  - ✅ 差值 < 3°（正常）
  - ⚠️  差值 3-5°（注意）
  - ❌ 差值 > 5°（异常）

**典型使用场景：**
- 诊断旋转时激光扫描与地图偏移的问题
- 验证EKF配置是否合理
- 检查传感器融合是否存在延迟

---

### 4. slam_monitor.py
**功能：** SLAM建图实时监控工具（综合监控）

**用途：**
- 实时监控map→odom坐标系变换（角度偏移、距离）
- 实时显示机器人在地图中的位置和朝向
- 对比EKF、编码器、IMU的yaw角度
- 检测建图过程中的累积误差
- 验证多传感器融合效果

**使用方法：**
```bash
cd /home/nvidia/robot_ws
python3 test_scripts/slam_monitor.py
```

**说明：**
- 订阅话题：
  - `/tf` (map→odom变换)
  - `/odometry/filtered` (EKF融合数据)
  - `/odom` (编码器数据)
  - `/imu/data_raw` (IMU数据)
- 显示内容：
  - 🗺️ map→odom角度偏移和距离
  - 🤖 机器人距离起点和朝向角度
  - 📊 EKF、编码器、IMU的yaw角度
  - Δ 各数据源之间的角度差值

**典型使用场景：**
- 建图时实时监控精度
- 检查slam_toolbox的回环检测效果
- 诊断累积误差来源
- 验证传感器配置是否合理

---

### 5. USB稳定别名与udev规则（强烈推荐）
为避免重启后 `/dev/ttyCH341USBx` 编号变化，本项目提供稳定别名：
- IMU: `/dev/imu_usb`
- 编码器/里程计: `/dev/encoder_usb`
- 速度指令: `/dev/cmd_usb`

安装/刷新规则（需要 sudo）：
```bash
sudo cp /home/nvidia/robot_ws/imu_usb.rules /etc/udev/rules.d/99-robot-usb.rules
sudo udevadm control --reload
sudo udevadm trigger
```
或运行提示脚本：
```bash
bash /home/nvidia/robot_ws/test_scripts/install_udev_rules.sh
```
验证别名：
```bash
ls -l /dev/imu_usb /dev/encoder_usb /dev/cmd_usb
```
说明：
- 规则使用物理口位（KERNELS）绑定到具体 Hub 端口；若更换 Hub 或改线，请用 `udevadm info -a -n /dev/ttyCH341USBx` 更新 `/home/nvidia/robot_ws/imu_usb.rules` 中的 KERNELS 值。
- 源码与 launch 已改为默认使用上述别名，安装规则后可直接使用。

排错：
- 别名未出现 → 重插设备或执行 `sudo udevadm trigger`
- 仍无效 → 用 `udevadm info -a -n /dev/ttyCH341USBx` 核对 `KERNELS` 是否匹配当前接线，再更新 `imu_usb.rules`
- 接线变化后 → 运行 `check_usb_ports_v2.sh --slow` 确认别名映射是否正确

---

## 注意事项

1. 这些脚本是**独立工具**，不会被自动启动
2. 使用前确保已运行 `source install/setup.bash`
3. 运行时需要相关ROS2节点（如IMU节点）已启动

---

## 历史记录

- 2025-11-06: 创建 `imu_angle_monitor.py` - IMU角度监控工具
- 2025-11-07: 移除 `particle_cloud_to_pose_array.py`（已集成到包内，见 `src/robot_bringup_cpp/scripts/`）
- 2025-11-07: 创建 `odom_ekf_compare.py` - 编码器vs EKF角度对比工具
- 2025-11-07: 创建 `slam_monitor.py` - SLAM建图综合监控工具
- 2025-11-09: 创建 `check_usb_ports.sh` - USB端口检查工具（开机检查）
- 2025-11-10: 新增 udev 规则与稳定别名说明，提供安装脚本 `install_udev_rules.sh`
- 2025-11-10: 升级为 `check_usb_ports_v2.sh`（串口原始模式、滑动窗口识别、慢速模式、别名优先）

