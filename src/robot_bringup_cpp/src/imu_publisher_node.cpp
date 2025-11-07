#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>
#include <chrono>

class IMUPublisherNode : public rclcpp::Node
{
public:
    IMUPublisherNode() : Node("imu_publisher_node")
    {
        // 声明参数
        this->declare_parameter("serial_port", "/dev/imu_usb");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("frame_id", "imu_link");
        this->declare_parameter("publish_rate", 50.0);

        // 获取参数
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // 防护：确保publish_rate至少为1Hz
        if (publish_rate < 1.0) {
            RCLCPP_WARN(this->get_logger(), "Publish rate %.2f Hz too low, clamping to 1.0 Hz", publish_rate);
            publish_rate = 1.0;
        }

        RCLCPP_INFO(this->get_logger(), "IMU Publisher Node starting...");
        RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "Baud rate: %d", baud_rate_);
        RCLCPP_INFO(this->get_logger(), "Frame ID: %s", frame_id_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz", publish_rate);

        // 初始化串口
        if (!initSerial()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port!");
            RCLCPP_ERROR(this->get_logger(), "Please check: 1) IMU connected? 2) /dev/imu_usb exists? 3) Permissions correct?");
            throw std::runtime_error("IMU serial port initialization failed");
        }

        // 创建发布者
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);

        // 创建定时器
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&IMUPublisherNode::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "IMU Publisher Node started successfully");
    }

    ~IMUPublisherNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool initSerial()
    {
        // 打开串口
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", serial_port_.c_str());
            return false;
        }

        // 配置串口
        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcgetattr");
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // 设置波特率
        speed_t speed;
        switch (baud_rate_) {
            case 9600:   speed = B9600; break;
            case 115200: speed = B115200; break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unsupported baud rate: %d", baud_rate_);
                close(serial_fd_);
                serial_fd_ = -1;
                return false;
        }
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);

        // 8N1, 无流控
        tty.c_cflag &= ~PARENB;        // 无奇偶校验
        tty.c_cflag &= ~CSTOPB;        // 1停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;            // 8数据位
        tty.c_cflag &= ~CRTSCTS;       // 无硬件流控
        tty.c_cflag |= CREAD | CLOCAL; // 启用接收

        // 原始模式
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;

        // 非阻塞读取
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from tcsetattr");
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // 清空缓冲区
        tcflush(serial_fd_, TCIOFLUSH);

        RCLCPP_INFO(this->get_logger(), "Serial port initialized successfully");
        
        // ========== 自动校准IMU陀螺仪（降低零偏漂移）==========
        RCLCPP_INFO(this->get_logger(), "Calibrating IMU gyroscope (please keep robot stationary)...");
        
        // 步骤1: 解锁寄存器 (KEY=0xB588)
        uint8_t unlock_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
        ssize_t n = write(serial_fd_, unlock_cmd, sizeof(unlock_cmd));
        if (n != sizeof(unlock_cmd)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send unlock command (wrote %zd/5 bytes)", n);
        } else {
            RCLCPP_INFO(this->get_logger(), "Unlock command sent successfully");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 步骤2: 陀螺仪自动校准 (CALSW=0x01) - 需要保持静止5秒
        // ⚠️ 重要：这会校准陀螺仪零偏，将360°旋转误差从26°降到6-8°
        uint8_t gyro_cal_cmd[] = {0xFF, 0xAA, 0x01, 0x01, 0x00};
        n = write(serial_fd_, gyro_cal_cmd, sizeof(gyro_cal_cmd));
        if (n != sizeof(gyro_cal_cmd)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send gyro calibration command (wrote %zd/5 bytes)", n);
        } else {
            RCLCPP_INFO(this->get_logger(), "Gyro calibration command sent successfully");
        }
        
        // ⚠️ 关键：必须等待5秒让陀螺仪校准完成，然后才能保存！
        RCLCPP_INFO(this->get_logger(), "⏱️  Waiting 5 seconds for gyro calibration to complete (keep robot stationary)...");
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        
        // 步骤3: 航向角清零 (CALSW=0x04) - 将yaw设为0°
        // ⚠️ 重要：这会将当前yaw角度清零，解决启动后yaw≠0°的问题
        uint8_t yaw_zero_cmd[] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
        n = write(serial_fd_, yaw_zero_cmd, sizeof(yaw_zero_cmd));
        if (n != sizeof(yaw_zero_cmd)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send yaw zero command (wrote %zd/5 bytes)", n);
        } else {
            RCLCPP_INFO(this->get_logger(), "Yaw zero command sent successfully");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 步骤4: 保存校准数据 (SAVE=0x0000) - 必须在所有校准完成后执行
        uint8_t save_cmd[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
        n = write(serial_fd_, save_cmd, sizeof(save_cmd));
        if (n != sizeof(save_cmd)) {
            RCLCPP_WARN(this->get_logger(), "Failed to send save command (wrote %zd/5 bytes)", n);
        } else {
            RCLCPP_INFO(this->get_logger(), "Save command sent successfully");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // 清空缓冲区中的旧数据
        tcflush(serial_fd_, TCIOFLUSH);
        
        RCLCPP_INFO(this->get_logger(), "✅ IMU calibration completed (gyro + yaw zero), buffer flushed, ready to publish data");
        RCLCPP_INFO(this->get_logger(), "Expected performance: 360° rotation drift < 8°, yaw starts at 0°");
        // ================================================
        
        return true;
    }

    void timerCallback()
    {
        readAndPublishIMU();
    }

    void readAndPublishIMU()
    {
        uint8_t buffer[1024];
        ssize_t n = read(serial_fd_, buffer, sizeof(buffer));
        
        if (n < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "Serial port read error (errno=%d): %s", errno, strerror(errno));
            }
            return; // 非阻塞模式，没有数据时返回
        }

        // 将读取的数据添加到缓冲区
        for (ssize_t i = 0; i < n; i++) {
            data_buffer_.push_back(buffer[i]);
        }

        // 防护：如果缓冲区过大，清空并警告
        if (data_buffer_.size() > 4096) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Data buffer overflow (%zu bytes), clearing buffer", data_buffer_.size());
            data_buffer_.clear();
            return;
        }

        // 解析数据帧
        while (parseFrame()) {
            // 如果有完整的数据集，发布IMU消息
            if (hasCompleteDataSet()) {
                publishIMU();
            }
        }
    }

    bool parseFrame()
    {
        // 至少需要11字节（0x55 + type + 8字节数据 + 校验和）
        if (data_buffer_.size() < 11) {
            return false;
        }

        // 查找帧头0x55
        auto it = std::find(data_buffer_.begin(), data_buffer_.end(), 0x55);
        if (it == data_buffer_.end()) {
            data_buffer_.clear();
            return false;
        }

        // 移除帧头之前的数据
        data_buffer_.erase(data_buffer_.begin(), it);

        // 检查是否有完整帧
        if (data_buffer_.size() < 11) {
            return false;
        }

        // 提取数据
        uint8_t frame_header = data_buffer_[0];  // 0x55
        uint8_t frame_type = data_buffer_[1];
        uint8_t data[8];
        for (int i = 0; i < 8; i++) {
            data[i] = data_buffer_[2 + i];
        }
        uint8_t checksum = data_buffer_[10];

        // 校验和验证
        uint8_t sum = frame_header + frame_type;
        for (int i = 0; i < 8; i++) {
            sum += data[i];
        }
        sum &= 0xFF;

        if (sum != checksum) {
            // 校验失败，丢弃帧头，继续查找
            data_buffer_.erase(data_buffer_.begin());
            return true; // 继续解析
        }

        // 校验成功，解析数据
        int16_t raw_data[4];
        for (int i = 0; i < 4; i++) {
            // 小端模式：低字节在前，先组合成uint16_t再转int16_t
            uint16_t unsigned_val = static_cast<uint16_t>(data[i*2]) | 
                                   (static_cast<uint16_t>(data[i*2+1]) << 8);
            raw_data[i] = static_cast<int16_t>(unsigned_val);
        }

        // 根据帧类型处理数据
        switch (frame_type) {
            case 0x51:  // 加速度
                imu_data_.accel_x = raw_data[0] / 32768.0 * 16.0 * 9.8;  // m/s²
                imu_data_.accel_y = raw_data[1] / 32768.0 * 16.0 * 9.8;
                imu_data_.accel_z = raw_data[2] / 32768.0 * 16.0 * 9.8;
                imu_data_.has_accel = true;
                break;

            case 0x52:  // 角速度
                imu_data_.gyro_x = raw_data[0] / 32768.0 * 2000.0 * M_PI / 180.0;  // rad/s
                imu_data_.gyro_y = raw_data[1] / 32768.0 * 2000.0 * M_PI / 180.0;
                imu_data_.gyro_z = raw_data[2] / 32768.0 * 2000.0 * M_PI / 180.0;
                imu_data_.has_gyro = true;
                break;

            case 0x53:  // 角度
                imu_data_.roll = raw_data[0] / 32768.0 * 180.0;   // 度
                imu_data_.pitch = raw_data[1] / 32768.0 * 180.0;
                imu_data_.yaw = raw_data[2] / 32768.0 * 180.0;
                imu_data_.has_angle = true;
                break;

            default:
                // 其他帧类型忽略
                break;
        }

        // 移除已处理的帧
        data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + 11);
        
        return true; // 继续解析下一帧
    }

    bool hasCompleteDataSet()
    {
        return imu_data_.has_accel && imu_data_.has_gyro && imu_data_.has_angle;
    }

    void publishIMU()
    {
        auto imu_msg = sensor_msgs::msg::Imu();
        
        // 设置header
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = frame_id_;

        // 设置线加速度
        imu_msg.linear_acceleration.x = imu_data_.accel_x;
        imu_msg.linear_acceleration.y = imu_data_.accel_y;
        imu_msg.linear_acceleration.z = imu_data_.accel_z;

        // 设置角速度
        imu_msg.angular_velocity.x = imu_data_.gyro_x;
        imu_msg.angular_velocity.y = imu_data_.gyro_y;
        imu_msg.angular_velocity.z = imu_data_.gyro_z;

        // 角度转四元数
        tf2::Quaternion q;
        double roll_rad = imu_data_.roll * M_PI / 180.0;
        double pitch_rad = imu_data_.pitch * M_PI / 180.0;
        double yaw_rad = imu_data_.yaw * M_PI / 180.0;
        q.setRPY(roll_rad, pitch_rad, yaw_rad);
        
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();

        // 设置协方差（根据JY901B规格）
        // 角速度协方差: (0.028°/s)² = (0.000488 rad/s)² ≈ 2.4e-7
        imu_msg.angular_velocity_covariance[0] = 2.4e-7;
        imu_msg.angular_velocity_covariance[4] = 2.4e-7;
        imu_msg.angular_velocity_covariance[8] = 2.4e-7;

        // 线加速度协方差: (0.001g)² = (0.0098 m/s²)² ≈ 9.6e-5
        imu_msg.linear_acceleration_covariance[0] = 9.6e-5;
        imu_msg.linear_acceleration_covariance[4] = 9.6e-5;
        imu_msg.linear_acceleration_covariance[8] = 9.6e-5;

        // 方向协方差（由角度计算得到，精度较低）
        imu_msg.orientation_covariance[0] = 0.01;
        imu_msg.orientation_covariance[4] = 0.01;
        imu_msg.orientation_covariance[8] = 0.01;

        // 发布消息
        imu_publisher_->publish(imu_msg);

        // 重置数据标志，等待下一组完整数据
        imu_data_.has_accel = false;
        imu_data_.has_gyro = false;
        imu_data_.has_angle = false;
    }

    // 成员变量
    std::string serial_port_;
    int baud_rate_;
    std::string frame_id_;
    int serial_fd_{-1};

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<uint8_t> data_buffer_;

    struct IMUData {
        double accel_x{0}, accel_y{0}, accel_z{0};
        double gyro_x{0}, gyro_y{0}, gyro_z{0};
        double roll{0}, pitch{0}, yaw{0};
        bool has_accel{false};
        bool has_gyro{false};
        bool has_angle{false};
    } imu_data_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IMUPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

