# 测试与调试工具

本目录包含用于机器人系统测试和调试的独立脚本。

## 工具列表

### 1. imu_angle_monitor.py
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

### 2. odom_ekf_compare.py
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

### 3. slam_monitor.py
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

