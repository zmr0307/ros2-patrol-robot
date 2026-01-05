//serial.h
#ifndef __SERIAL_H
#define __SERIAL_H

#include <stdio.h>
#include <stdint.h>

// 初始化
void Serial_Init(void);

// 发送
void Serial_SendByte(uint8_t Byte);
void Serial_SendString(char *String);

// 接收
uint8_t Serial_GetRxFlag(void);      // 是否收到完整帧
uint8_t* Serial_GetRxBuffer(void);   // 获取接收缓冲区







// ========== 新增：里程计数据发送 ==========
/**
  * @brief  发送里程计数据帧（10字节）
  * @param  seq: 序列号（用于检测丢包）
  * @param  delta_left: 左轮编码器增量（脉冲数）
  * @param  delta_right: 右轮编码器增量（脉冲数）
  * @note   数据帧格式：[0xA5][seq 4B][delta_L 2B][delta_R 2B][checksum 1B]
  */
void Serial_SendOdomFrame(uint32_t seq, int16_t delta_left, int16_t delta_right);

/**
  * @brief  测试函数：发送测试里程计帧
  * @note   通过串口2发送3帧测试数据，验证格式
  */
void Serial_Test_SendOdom(void);

#endif

