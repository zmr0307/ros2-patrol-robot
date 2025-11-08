/**
 * @file serial_bridge_node.cpp
 * @brief 串口桥接节点 - 第三步：初始化串口并打包发送数据
 *
 * 功能：
 * 1. 订阅 /cmd_vel 话题，接收机器人的速度指令
 * 2. 执行逆运动学解算，将整体速度转换为左右轮的角速度
 * 3. 初始化串口设备（支持参数配置）
 * 4. 将轮速数据打包成10字节帧格式并通过串口发送
 *
 * 数据帧格式（10字节）：
 * +--------+-------------+-------------+----------+
 * | 帧头   | 左轮速度    | 右轮速度    | 校验和   |
 * | 0xA5   | 4 bytes     | 4 bytes     | 1 byte   |
 * | (1B)   | (float)     | (float)     | (uint8)  |
 * +--------+-------------+-------------+----------+
 *
 * 校验和计算：第1-8字节（左右轮速度）的累加和（uint8_t）
 *
 * 编译命令：
 *   cd ~/ros2_ws
 *   colcon build --packages-select robot_bringup_cpp
 *
 * 运行命令（默认参数）：
 *   source install/setup.bash
 *   ros2 run robot_bringup_cpp serial_bridge_node
 *
 * 运行命令（自定义串口参数）：
 *   ros2 run robot_bringup_cpp serial_bridge_node \
 *     --ros-args \
 *     -p serial_port:=/dev/ttyUSB0 \
 *     -p baud_rate:=115200
 *
 * 测试命令（在另一个终端）：
 *   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
 *     "{linear: {x: 0.5}, angular: {z: 0.0}}" --once
 *
 * 串口调试工具：
 *   sudo apt install minicom
 *   sudo minicom -D /dev/ttyUSB0 -b 115200
 *
 * @author robot_bringup_cpp 项目
 * @date 2025-10-03
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// 串口通信相关头文件
#include <string>
#include <vector>
#include <termios.h>  // 用于串口配置
#include <fcntl.h>    // 用于文件控制，如 O_RDWR
#include <unistd.h>   // 用于 read, write, close
#include <cstring>    // 用于 memcpy
#include <cmath>      // 用于 M_PI, std::abs, std::copysign
#include <thread>     // 用于延时
#include <chrono>     // 用于时间控制
#include <stdexcept>

// 通信协议常量
const uint8_t FRAME_HEADER = 0xA5;  // 帧头标识

/**
 * @class SerialBridgeNode
 * @brief 串口桥接节点类
 *
 * 完整实现 ROS 2 到串口的数据桥接：
 * - 接收 /cmd_vel 速度指令
 * - 执行差分驱动逆运动学解算
 * - 打包数据并通过串口发送到下位机
 */
class SerialBridgeNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     *
     * 初始化流程：
     * 1. 声明并获取 ROS 参数
     * 2. 初始化串口设备
     * 3. 创建 /cmd_vel 订阅者
     */
    SerialBridgeNode() : Node("serial_bridge_node")
    {
        // =================================================================
        // 第一步：声明 ROS 参数
        // =================================================================
        this->declare_parameter("serial_port", "/dev/ttyCH341USB1");  // 修正：重新上电后映射为USB1
        this->declare_parameter("baud_rate", 115200);

        // =================================================================
        // 第二步：获取参数值
        // =================================================================
        serial_port_name_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();

        // 打印节点启动信息
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "🤖 串口桥接节点已启动");
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "📊 机器人参数:");
        RCLCPP_INFO(this->get_logger(), "   轮距 = %.3f m", WHEEL_SEPARATION);
        RCLCPP_INFO(this->get_logger(), "   轮径 = %.4f m", WHEEL_RADIUS);
        RCLCPP_INFO(this->get_logger(), "🛡️  速度限制 (8 RPM max):");
        RCLCPP_INFO(this->get_logger(), "   最大线速度 = %.4f m/s (%.2f cm/s)",
                    MAX_LINEAR_VEL, MAX_LINEAR_VEL * 100);
        RCLCPP_INFO(this->get_logger(), "   最大角速度 = %.4f rad/s (%.2f °/s)",
                    MAX_ANGULAR_VEL, MAX_ANGULAR_VEL * 180 / M_PI);
        RCLCPP_INFO(this->get_logger(), "   最大轮速 = %.4f rad/s (8 RPM)", MAX_WHEEL_RAD_S);
        RCLCPP_INFO(this->get_logger(), "📡 串口参数:");
        RCLCPP_INFO(this->get_logger(), "   端口 = %s", serial_port_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "   波特率 = %d", baud_rate_);
        RCLCPP_INFO(this->get_logger(), "========================================");

        // =================================================================
        // 第三步：初始化串口
        // =================================================================
        init_serial();

        // =================================================================
        // 第四步：创建 /cmd_vel 订阅者
        // =================================================================
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",
            10,
            std::bind(&SerialBridgeNode::cmd_vel_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "✅ 已订阅话题: /cmd_vel");

        // =================================================================
        // 第五步：安全看门狗（已禁用）
        // =================================================================
        // 看门狗已被禁用，机器人将保持最后收到的速度指令
        // 只有通过键盘控制节点按下 'k' 键发送 0 速度指令才会停止
        
        // 注释：如果需要重新启用看门狗，取消下面代码的注释
        /*
        last_cmd_time_ = this->now();
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SerialBridgeNode::watchdog_callback, this)
        );
        RCLCPP_INFO(this->get_logger(), "🛡️  安全看门狗已配置:");
        RCLCPP_INFO(this->get_logger(), "   超时阈值 = %.1f 秒", CMD_TIMEOUT);
        */

        RCLCPP_WARN(this->get_logger(), "⚠️  安全看门狗已禁用！");
        RCLCPP_WARN(this->get_logger(), "   机器人将保持最后收到的速度指令");
        RCLCPP_WARN(this->get_logger(), "   必须手动按 'k' 键发送停止指令");
        RCLCPP_WARN(this->get_logger(), "   ⚠️  请注意安全！如遇紧急情况，请按 'k' 键停止");

        RCLCPP_INFO(this->get_logger(), "⏳ 等待接收速度指令...");
        RCLCPP_INFO(this->get_logger(), "========================================");
    }

    /**
     * @brief 析构函数
     *
     * 在节点关闭时，确保正确关闭串口设备。
     */
    ~SerialBridgeNode()
    {
        if (serial_fd_ != -1)
        {
            close(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "🔌 串口已关闭");
        }
    }

private:
    // ==========================================================================
    // 机器人物理参数（必须与 diff_drive_controller.yaml 一致）
    // ==========================================================================
  static constexpr double WHEEL_SEPARATION = 0.23364;   // 轮距 (m) - 旋转标定线性插值（2圈少转10.5°，3圈多转6.1°）
  static constexpr double WHEEL_RADIUS = 0.06475;     // 轮径 (m) - 直线标定11cm（实际11.0cm，里程计10.8cm）

    // ==========================================================================
    // 速度限制（基于8 RPM最大轮速）
    // ==========================================================================
    // 最大轮速: 8 RPM = 0.8378 rad/s
    // 最大线速度: v = ω * R = 0.8378 * 0.06475 = 0.05424 m/s (实测保守值0.05362 m/s)
    // 最大角速度: ω = 2 * v / L = 2 * 0.05362 / 0.23364 = 0.4590 rad/s (26.29°/s)
    static constexpr double MAX_WHEEL_RAD_S = 0.8378;   // rad/s (8 RPM限制)
    static constexpr double MAX_LINEAR_VEL = 0.05362;   // m/s (实测保守值)
    static constexpr double MAX_ANGULAR_VEL = 0.4590;   // rad/s (旋转标定：2*0.05362/0.23364)

    // ==========================================================================
    // ROS 2 成员变量
    // ==========================================================================
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;  // 看门狗定时器

    // ==========================================================================
    // 串口相关成员变量
    // ==========================================================================
    std::string serial_port_name_;  // 串口设备名（如 /dev/ttyUSB0）
    int baud_rate_;                 // 波特率（如 115200）
    int serial_fd_ = -1;            // 串口文件描述符（-1 表示未打开）

    // ==========================================================================
    // 安全保护相关成员变量
    // ==========================================================================
    rclcpp::Time last_cmd_time_;              // 最后一次收到指令的时间
    static constexpr double CMD_TIMEOUT = 0.5; // 指令超时时间（秒）
    bool emergency_stop_sent_ = false;        // 是否已发送急停指令
    bool watchdog_activated_ = false;         // 看门狗是否已激活（收到第一个指令后激活）

    // ==========================================================================
    // 串口初始化函数
    // ==========================================================================

    /**
     * @brief 初始化串口设备
     *
     * 执行标准的 Linux 串口初始化流程：
     * 1. 打开串口设备文件
     * 2. 配置串口参数（波特率、数据位、校验位、停止位）
     * 3. 设置为原始数据模式（raw mode）
     *
     * 配置说明：
     * - 数据位：8 位
     * - 校验位：无校验
     * - 停止位：1 位
     * - 流控制：无流控制
     */
    void init_serial()
    {
        RCLCPP_INFO(this->get_logger(), "🔧 正在初始化串口...");

        // ================================================================
        // 第一步：打开串口设备
        // ================================================================
        // O_RDWR: 读写模式
        // O_NOCTTY: 不将此设备设为控制终端
        serial_fd_ = open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY);

        if (serial_fd_ == -1)
        {
            RCLCPP_FATAL(this->get_logger(),
                        "❌ 无法打开串口: %s",
                        serial_port_name_.c_str());
            RCLCPP_FATAL(this->get_logger(),
                        "   错误原因: %s",
                        strerror(errno));
            RCLCPP_FATAL(this->get_logger(),
                        "   请检查：");
            RCLCPP_FATAL(this->get_logger(),
                        "   1. 设备是否已连接");
            RCLCPP_FATAL(this->get_logger(),
                        "   2. 是否有访问权限（sudo chmod 666 /dev/ttyCH341USB*）");
            RCLCPP_FATAL(this->get_logger(),
                        "   3. 端口名称是否正确");
            RCLCPP_FATAL(this->get_logger(), "🔴 节点将退出");
            rclcpp::shutdown();
            throw std::runtime_error("Serial port initialization failed");
        }

        // ================================================================
        // 第二步：配置串口参数
        // ================================================================
        struct termios options;

        // 获取当前串口配置
        if (tcgetattr(serial_fd_, &options) != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "❌ 无法获取串口配置");
            close(serial_fd_);
            serial_fd_ = -1;
            RCLCPP_FATAL(this->get_logger(), "🔴 节点将退出");
            rclcpp::shutdown();
            throw std::runtime_error("Serial port initialization failed");
        }

        // 设置波特率（输入和输出都设置为相同波特率）
        speed_t baud;
        switch (baud_rate_)
        {
            case 9600:    baud = B9600;    break;
            case 19200:   baud = B19200;   break;
            case 38400:   baud = B38400;   break;
            case 57600:   baud = B57600;   break;
            case 115200:  baud = B115200;  break;
            case 230400:  baud = B230400;  break;
            case 460800:  baud = B460800;  break;
            case 921600:  baud = B921600;  break;
            default:
                RCLCPP_FATAL(this->get_logger(),
                            "❌ 不支持的波特率: %d",
                            baud_rate_);
                close(serial_fd_);
                serial_fd_ = -1;
                RCLCPP_FATAL(this->get_logger(), "🔴 节点将退出");
                rclcpp::shutdown();
                throw std::runtime_error("Unsupported baud rate");
        }

        cfsetispeed(&options, baud);  // 设置输入波特率
        cfsetospeed(&options, baud);  // 设置输出波特率

        // 配置为 8N1 模式（8个数据位, 无校验, 1个停止位）
        options.c_cflag &= ~PARENB;   // 禁用校验位
        options.c_cflag &= ~CSTOPB;   // 1个停止位
        options.c_cflag &= ~CSIZE;    // 清除数据位掩码
        options.c_cflag |= CS8;       // 8个数据位
        options.c_cflag &= ~CRTSCTS;  // 禁用硬件流控制
        options.c_cflag |= CREAD | CLOCAL;  // 启用接收器，忽略调制解调器控制线

        // 设置为原始数据模式（raw mode）
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 禁用规范模式和回显
        options.c_iflag &= ~(IXON | IXOFF | IXANY);          // 禁用软件流控制
        options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        options.c_oflag &= ~OPOST;  // 禁用输出处理

        // 设置读取超时（非阻塞模式）
        options.c_cc[VMIN] = 0;   // 最小读取字符数
        options.c_cc[VTIME] = 1;  // 读取超时时间（0.1秒）

        // 应用配置
        if (tcsetattr(serial_fd_, TCSANOW, &options) != 0)
        {
            RCLCPP_FATAL(this->get_logger(), "❌ 无法设置串口配置");
            close(serial_fd_);
            serial_fd_ = -1;
            RCLCPP_FATAL(this->get_logger(), "🔴 节点将退出");
            rclcpp::shutdown();
            throw std::runtime_error("Serial port initialization failed");
        }

        // ================================================================
        // 第三步：清空串口缓冲区
        // ================================================================
        tcflush(serial_fd_, TCIOFLUSH);

        RCLCPP_INFO(this->get_logger(),
                    "✅ 串口 %s 已成功打开，波特率 %d",
                    serial_port_name_.c_str(),
                    baud_rate_);
    }

    // ==========================================================================
    // 回调函数
    // ==========================================================================

    /**
     * @brief /cmd_vel 话题回调函数
     *
     * 处理流程：
     * 1. 提取速度指令（线速度 v 和角速度 ω）
     * 2. 执行逆运动学解算，计算左右轮角速度
     * 3. 打包数据为 10 字节帧格式
     * 4. 通过串口发送数据
     *
     * @param msg 接收到的 Twist 消息
     */
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // ================================================================
        // 看门狗相关代码已禁用
        // ================================================================
        // 注释：如果重新启用看门狗，取消下面代码的注释
        /*
        last_cmd_time_ = this->now();
        emergency_stop_sent_ = false;
        if (!watchdog_activated_) {
            watchdog_activated_ = true;
            RCLCPP_INFO(this->get_logger(), "✅ 收到第一个指令，看门狗已激活");
        }
        */

        // ================================================================
        // 第一步：提取速度指令并进行安全限制
        // ================================================================
        double v = msg->linear.x;       // 线速度 (m/s)
        double omega = msg->angular.z;  // 角速度 (rad/s)

        // 🛡️ 安全保护1：钳制输入速度
        bool limited = false;
        if (std::abs(v) > MAX_LINEAR_VEL) {
            RCLCPP_WARN(this->get_logger(),
                       "⚠️  线速度超限! %.4f -> %.4f m/s",
                       v, std::copysign(MAX_LINEAR_VEL, v));
            v = std::copysign(MAX_LINEAR_VEL, v);
            limited = true;
        }
        if (std::abs(omega) > MAX_ANGULAR_VEL) {
            RCLCPP_WARN(this->get_logger(),
                       "⚠️  角速度超限! %.4f -> %.4f rad/s",
                       omega, std::copysign(MAX_ANGULAR_VEL, omega));
            omega = std::copysign(MAX_ANGULAR_VEL, omega);
            limited = true;
        }

        // ================================================================
        // 第二步：执行逆运动学解算
        // ================================================================
        // 差分驱动机器人的逆运动学公式：
        //   ω_left  = (v - ω * L/2) / R
        //   ω_right = (v + ω * L/2) / R
        float left_wheel_rad_s = static_cast<float>(
            (v - omega * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        );
        float right_wheel_rad_s = static_cast<float>(
            (v + omega * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        );

        // 🛡️ 安全保护2：钳制轮速（双保险）
        if (std::abs(left_wheel_rad_s) > MAX_WHEEL_RAD_S) {
            RCLCPP_WARN(this->get_logger(),
                       "⚠️  左轮速度超限! %.4f -> %.4f rad/s",
                       left_wheel_rad_s, std::copysign(MAX_WHEEL_RAD_S, left_wheel_rad_s));
            left_wheel_rad_s = static_cast<float>(std::copysign(MAX_WHEEL_RAD_S, left_wheel_rad_s));
            limited = true;
        }
        if (std::abs(right_wheel_rad_s) > MAX_WHEEL_RAD_S) {
            RCLCPP_WARN(this->get_logger(),
                       "⚠️  右轮速度超限! %.4f -> %.4f rad/s",
                       right_wheel_rad_s, std::copysign(MAX_WHEEL_RAD_S, right_wheel_rad_s));
            right_wheel_rad_s = static_cast<float>(std::copysign(MAX_WHEEL_RAD_S, right_wheel_rad_s));
            limited = true;
        }

        // 简化日志：每1秒最多打印1次
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "📥 指令: v=%.3f m/s, ω=%.3f rad/s → 轮速: L=%.3f, R=%.3f rad/s%s",
                    v, omega, left_wheel_rad_s, right_wheel_rad_s,
                    limited ? " [限制]" : "");

        // ================================================================
        // 第三步：打包数据
        // ================================================================
        // 数据帧格式（10字节）：
        // [0xA5][左轮速度 4B][右轮速度 4B][校验和 1B]
        std::vector<uint8_t> buffer;

        // 1. 添加帧头
        buffer.push_back(FRAME_HEADER);

        // 2. 添加左轮速度（4字节 float）
        uint8_t* left_bytes = reinterpret_cast<uint8_t*>(&left_wheel_rad_s);
        for (int i = 0; i < 4; i++)
        {
            buffer.push_back(left_bytes[i]);
        }

        // 3. 添加右轮速度（4字节 float）
        uint8_t* right_bytes = reinterpret_cast<uint8_t*>(&right_wheel_rad_s);
        for (int i = 0; i < 4; i++)
        {
            buffer.push_back(right_bytes[i]);
        }

        // 4. 计算校验和（第1-8字节的累加和）
        uint8_t checksum = 0;
        for (size_t i = 1; i < buffer.size(); i++)  // 从索引1开始，跳过帧头
        {
            checksum += buffer[i];
        }
        buffer.push_back(checksum);

        // ================================================================
        // 第四步：通过串口发送数据
        // ================================================================
        if (serial_fd_ != -1)
        {
            ssize_t bytes_written = write(serial_fd_, buffer.data(), buffer.size());

            if (bytes_written == -1)
            {
                RCLCPP_ERROR(this->get_logger(),
                            "❌ 串口发送失败: %s",
                            strerror(errno));
            }
            else if (bytes_written != static_cast<ssize_t>(buffer.size()))
            {
                RCLCPP_WARN(this->get_logger(),
                           "⚠️  数据发送不完整: 期望%zu字节，实际%zd字节",
                           buffer.size(), bytes_written);
            }
            // 发送成功，无需日志（避免刷屏）
        }
        else
        {
            RCLCPP_WARN(this->get_logger(),
                       "⚠️  串口未打开，无法发送数据");
        }
    }

    // ==========================================================================
    // 安全看门狗回调函数
    // ==========================================================================

    /**
     * @brief 看门狗定时器回调函数
     *
     * 定期检查是否超时未收到指令：
     * - 如果超过 CMD_TIMEOUT 秒未收到新指令
     * - 连续发送2次零速度指令（间隔5ms），确保机器人可靠停止
     *
     * 这是一个关键的安全功能，防止：
     * - teleop_twist_keyboard 松开按键后不发送零速度
     * - 网络断开导致机器人失控
     * - 控制节点崩溃后机器人继续运动
     */
    void watchdog_callback()
    {
        // 如果看门狗还未激活（还没收到第一个指令），直接返回
        if (!watchdog_activated_) {
            return;
        }
        
        // 计算距离上次收到指令的时间
        double elapsed_time = (this->now() - last_cmd_time_).seconds();

        // 如果超时且尚未发送急停指令
        if (elapsed_time > CMD_TIMEOUT && !emergency_stop_sent_)
        {
            RCLCPP_WARN(this->get_logger(),
                       "⚠️  看门狗触发！%.2f 秒未收到指令 → 连续发送2次零速度指令",
                       elapsed_time);

            // 连续发送2次零速度指令，间隔5ms（提高可靠性）
            send_zero_velocity();
            RCLCPP_WARN(this->get_logger(), "   第1次零速度指令已发送，等待5ms...");
            
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            
            send_zero_velocity();
            RCLCPP_WARN(this->get_logger(), "   第2次零速度指令已发送");

            // 标记已发送急停，避免重复发送
            emergency_stop_sent_ = true;
        }
    }

    /**
     * @brief 发送零速度指令
     *
     * 立即发送零速度（左右轮速度都为0）到下位机，
     * 确保机器人停止运动。
     */
    void send_zero_velocity()
    {
        // 零速度（左右轮都是0 rad/s）
        float left_wheel_rad_s = 0.0f;
        float right_wheel_rad_s = 0.0f;

        // 打包数据帧
        std::vector<uint8_t> buffer;

        // 1. 添加帧头
        buffer.push_back(FRAME_HEADER);

        // 2. 添加左轮速度（4字节 float，值为0）
        uint8_t* left_bytes = reinterpret_cast<uint8_t*>(&left_wheel_rad_s);
        for (int i = 0; i < 4; i++)
        {
            buffer.push_back(left_bytes[i]);
        }

        // 3. 添加右轮速度（4字节 float，值为0）
        uint8_t* right_bytes = reinterpret_cast<uint8_t*>(&right_wheel_rad_s);
        for (int i = 0; i < 4; i++)
        {
            buffer.push_back(right_bytes[i]);
        }

        // 4. 计算校验和（前8个字节的累加和）
        uint8_t checksum = 0;
        for (size_t i = 1; i < buffer.size(); i++)
        {
            checksum += buffer[i];
        }
        buffer.push_back(checksum);

        // 通过串口发送（不打印详细日志，由调用者打印）
        if (serial_fd_ != -1)
        {
            ssize_t bytes_written = write(serial_fd_, buffer.data(), buffer.size());

            if (bytes_written != static_cast<ssize_t>(buffer.size()))
            {
                RCLCPP_ERROR(this->get_logger(), "❌ 零速度指令发送失败");
            }
        }
    }
};

/**
 * @brief 主函数
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码
 */
int main(int argc, char** argv)
{
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建节点实例
    auto node = std::make_shared<SerialBridgeNode>();

    // 运行节点
    rclcpp::spin(node);

    // 关闭 ROS 2
    rclcpp::shutdown();

    return 0;

}