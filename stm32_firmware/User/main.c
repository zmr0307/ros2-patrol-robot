/**
 * @file main.c
 * @brief STM32 ROS2 to Modbus Bridge (优化版 - 去除主循环调试信息)
 *
 * ============================================================================
 * 硬件说明 - HARDWARE NOTES
 * ============================================================================
 *
 * ⚠️ 【重要】机械装配特殊情况：
 *
 * 问题：左轮电机由于机械装配原因，安装方向与预期相反！（实测确认）
 *
 * 表现：
 *   - 发送正值给左轮 → 电机反转（向后）
 *   - 发送负值给左轮 → 电机正转（向前）
 *
 * 解决方案：
 *   - 在软件层面对左轮速度取反
 *   - 代码位置：main.c 主循环中 Modbus_SetSpeed_Sync() 调用前
 *
 * 验证方法：
 *   - 发送指令：左轮=+2 RPM, 右轮=+2 RPM
 *   - 预期结果：两轮都向前转，机器人直线前进
 *   - 实际发送：左轮=-2 RPM, 右轮=+2 RPM（软件自动取反左轮）
 *
 * ============================================================================
 * 优化说明 - OPTIMIZATION NOTES
 * ============================================================================
 *
 * 版本：v1.1 - 性能优化版
 *
 * 主要改动：
 * 1. 注释掉主循环中的所有printf（避免阻塞，防止丢帧）
 * 2. 保留启动时的初始化信息（只打印一次）
 * 3. 需要调试时，可以打开指定位置的注释
 *
 * 性能提升：
 * - 原版：处理1帧速度指令约12.6ms（大部分是printf）
 * - 优化后：处理1帧速度指令约1.1ms
 * - 结果：能跟上Jetson 20ms一次的速度指令下发
 *
 * ============================================================================
 */
#include "stm32f10x.h"
#include "serial.h"
#include "modbus.h"
#include "CRC.h"
#include <stdio.h>
#include <string.h>
#include "timer.h"
#include "Delay.h"

// 定义从 弧度/秒 (rad/s) 到 转/分钟 (RPM) 的转换常数
#define RADS_TO_RPM 9.54929658f

/**
 * @brief 解析来自Jetson的10字节数据帧
 */
uint8_t Parse_Frame(uint8_t *frame, float *left_speed_rads, float *right_speed_rads)
{
    uint8_t i;
    uint8_t checksum = 0;

    if (frame[0] != 0xA5)
        return 0;

    for (i = 1; i < 9; i++)
    {
        checksum += frame[i];
    }

    if (checksum != frame[9])
        return 0;

    uint8_t *left_ptr = (uint8_t *)left_speed_rads;
    uint8_t *right_ptr = (uint8_t *)right_speed_rads;

    for (i = 0; i < 4; i++)
    {
        left_ptr[i] = frame[1 + i];
        right_ptr[i] = frame[5 + i];
    }

    return 1;
}

int main(void)
{
    float left_speed_rads = 0.0f;
    float right_speed_rads = 0.0f;
    int16_t left_speed_rpm_10x = 0;
    int16_t right_speed_rpm_10x = 0;

    // ========== 里程计相关变量 ==========
    int32_t last_left_encoder = 0;
    int32_t last_right_encoder = 0;
    uint32_t odom_seq = 0;
    // ====================================

    // 初始化USART1(接收)和USART2(调试)
    Serial_Init();

    // 初始化USART3(Modbus)
    Modbus_Init();

    // ========== 启动信息（保留，只打印一次） ==========
    printf("\r\n========================================\r\n");
    printf("STM32 ROS2 to Modbus Bridge v1.1\r\n");
    printf("========================================\r\n");
    printf("Control Port (RX):  USART1 (PA9/PA10)\r\n");
    printf("Debug Port (TX):    USART2 (PA2/PA3)\r\n");
    printf("Modbus Port (TX):   USART3 (PB10/PB11)\r\n");
    printf("Baud Rate:          115200\r\n");
    printf("========================================\r\n");
    // ===================================================

    // 配置驱动器
    Modbus_InitConfig();

    printf("========================================\r\n");
    printf("System ready. Waiting for commands...\r\n\r\n");

    // ========== 初始化50Hz定时器 ==========
    Timer_Init();
    // ======================================

    while (1)
    {
        // ========== 50Hz里程计处理 ==========
        if (g_read_encoder_flag)
        {
            g_read_encoder_flag = 0;

            // 1. 读取当前编码器值
            int32_t current_left = Modbus_ReadEncoder_Left();

            int32_t current_right = Modbus_ReadEncoder_Right();

            // 2. 计算增量
            int16_t delta_left = (int16_t)(current_left - last_left_encoder);
            int16_t delta_right = (int16_t)(current_right - last_right_encoder);

            // 3. 发送里程计数据帧（通过USART2）
            Serial_SendOdomFrame(odom_seq, delta_left, delta_right);

            // 4. 更新last值
            last_left_encoder = current_left;
            last_right_encoder = current_right;

            // 5. 序列号递增
            odom_seq++;
        }
        // ====================================

        // ========== 速度指令接收处理 ==========
        if (Serial_GetRxFlag())
        {
            uint8_t *rx_buffer = Serial_GetRxBuffer();

            if (Parse_Frame(rx_buffer, &left_speed_rads, &right_speed_rads))
            {
                // 转换为 0.1 RPM 单位（乘以10）
                left_speed_rpm_10x = (int16_t)((left_speed_rads >= 0) ? (left_speed_rads * RADS_TO_RPM * 10.0f + 0.5f) : (left_speed_rads * RADS_TO_RPM * 10.0f - 0.5f));
                right_speed_rpm_10x = (int16_t)((right_speed_rads >= 0) ? (right_speed_rads * RADS_TO_RPM * 10.0f + 0.5f) : (right_speed_rads * RADS_TO_RPM * 10.0f - 0.5f));

                // ========== 调试信息（已注释，需要时打开） ==========
                // printf("OK L_rads=%+.2f R_rads=%+.2f -> L_rpm=%.1f R_rpm=%.1f\r\n",
                //        left_speed_rads,
                //        right_speed_rads,
                //        left_speed_rpm_10x / 10.0f,
                //        right_speed_rpm_10x / 10.0f);
                // ===================================================

                // 左轮取反（补偿机械安装方向）- 实测左轮方向相反
                int16_t left_speed_rpm_10x_reversed = -left_speed_rpm_10x;

                // 发送Modbus指令（通过USART3）
                Modbus_SetSpeed_Sync(left_speed_rpm_10x_reversed, right_speed_rpm_10x);
            }
            // ========== 调试信息（已注释，需要时打开） ==========
            // else
            // {
            //     printf("ERROR: Invalid Frame\r\n");
            // }
            // ===================================================
        }
        // ======================================
    }
}