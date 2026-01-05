#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdint.h>

// 函数声明
void Modbus_Init(void);
void Modbus_InitConfig(void);
void Modbus_SetSpeed_Sync(int16_t left_rpm, int16_t right_rpm);

// ========== 新增：编码器读取函数 ==========
// 读取左轮编码器值（32位累积值）
// 返回：编码器脉冲数（-2147483648 ~ 2147483647）
int32_t Modbus_ReadEncoder_Left(void);

// 读取右轮编码器值（32位累积值）
// 返回：编码器脉冲数（-2147483648 ~ 2147483647）
int32_t Modbus_ReadEncoder_Right(void);

// ========== 测试函数 ==========
// 测试Modbus读功能（使用假数据）
void Modbus_Test_ReadEncoder(void);
// ========== 编码器方向配置 ==========
/**
  * @brief  编码器方向配置
  * @note   根据实际测试结果修改这两个宏
  * 
  * ✅ 重要说明（已实测确认）：
  * 1. 左轮电机因机械装配原因，安装方向相反（实测：+2 RPM倒着走）
  * 2. 速度指令已经在发送时对左轮取反（见main.c第174-178行）
  * 3. 编码器安装在电机上，电机反向 → 编码器读数也需要反向
  * 4. 实测配置：LEFT_ENCODER_REVERSE = 1, RIGHT_ENCODER_REVERSE = 0
  * 5. 测试日期：2025年实际硬件测试
  * 
  * 验证方法：
  * - 发送指令：左轮+2 RPM，右轮+2 RPM
  * - 实际发送：左轮-2 RPM（取反），右轮+2 RPM
  * - 结果：机器人直线前进，左右编码器符号相同
  */
#define LEFT_ENCODER_REVERSE   1    // 左轮编码器是否取反：0=不取反，1=取反（实测需要反向）
#define RIGHT_ENCODER_REVERSE  0    // 右轮编码器是否取反：0=不取反，1=取反（实测不需要反向）

#endif

