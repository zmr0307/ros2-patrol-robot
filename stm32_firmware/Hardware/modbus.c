#include "modbus.h"
#include "CRC.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

/**
  * @file modbus.c
  * @brief Modbus通信模块 - 假数据开关版
  * @version v1.3
  * 
  * 重要配置开关:
  * USE_FAKE_ENCODER_DATA = 1 → 使用假数据，不连接Modbus硬件（不测试）
  * USE_FAKE_ENCODER_DATA = 0 → 使用真实Modbus通信，连接编码器硬件
  */

// ========== 测试开关（生产环境请设为0） ==========
#define USE_FAKE_ENCODER_DATA  0  // 1=假数据，0=真实Modbus（当前：测试模式）
// ===================================================

/**
  * @brief  初始化USART3用于Modbus通信
  * @param  None
  * @retval None
  */
void Modbus_Init(void)
{
    // 使能USART3和GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // USART3 TX (PB10) - 复用推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // USART3 RX (PB11) - 浮空输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置USART3参数
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);
    
    // 使能USART3
    USART_Cmd(USART3, ENABLE);
}

/**
  * @brief  通过USART3发送一个字节
  * @param  byte: 要发送的字节
  * @retval None
  */
static void USART3_SendByte(uint8_t byte)
{
    // 等待发送数据寄存器为空
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    // 发送数据
    USART_SendData(USART3, byte);
}

/**
  * @brief  发送单个Modbus写指令
  * @param  reg_addr: 寄存器地址
  * @param  value: 要写入的值
  * @retval None
  */
static void Modbus_WriteRegister(uint16_t reg_addr, uint16_t value)
{
    uint8_t frame[8];
    
    frame[0] = 0x01;                      // 设备地址
    frame[1] = 0x06;                      // 功能码：写单个寄存器
    frame[2] = (reg_addr >> 8) & 0xFF;    // 寄存器地址高字节
    frame[3] = reg_addr & 0xFF;           // 寄存器地址低字节
    frame[4] = (value >> 8) & 0xFF;       // 数据高字节
    frame[5] = value & 0xFF;              // 数据低字节
    
    uint16_t crc = Calculate_CRC16(frame, 6);
    frame[6] = crc & 0xFF;          // CRC低字节在前（Modbus标准）
    frame[7] = (crc >> 8) & 0xFF;  // CRC高字节在后（Modbus标准）
    
    // 通过串口2打印调试信息
    printf("    TX: ");
    for (uint8_t i = 0; i < 8; i++)
    {
        printf("%02X ", frame[i]);
    }
    printf("\r\n");
    
    // 通过串口3发送Modbus指令
    for (uint8_t i = 0; i < 8; i++)
    {
        USART3_SendByte(frame[i]);
    }
    
    // 等待发送完成
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    
    // 等待驱动器响应
    for (volatile uint32_t i = 0; i < 500000; i++);
}

/**
  * @brief  配置驱动器为速度模式，设置高精度
  * @param  None
  * @retval None
  */
void Modbus_InitConfig(void)
{
    printf("Configuring ZLAC8015D driver...\r\n");
    
    // 1. 设置速度模式
    printf("  [1] Set Speed Mode:\r\n");
    Modbus_WriteRegister(0x200D, 0x0003);
    
    // 2. 设置速度分辨率为 0.1 RPM
    printf("  [2] Set Resolution 0.1 RPM:\r\n");
    Modbus_WriteRegister(0x2022, 0x000A);
    
    // 3. 使能驱动器
    printf("  [3] Enable Driver:\r\n");
    Modbus_WriteRegister(0x200E, 0x0008);
    
    printf("Configuration complete!\r\n");
}

/**
  * @brief  生成并发送Modbus同步速度控制指令
  * @param  left_rpm: 左轮速度 (已乘以10，即发10代表1.0 RPM)
  * @param  right_rpm: 右轮速度 (已乘以10)
  * @retval None
  */
void Modbus_SetSpeed_Sync(int16_t left_rpm, int16_t right_rpm)
{
    // 1. 构建数据帧（11字节）
    uint8_t frame_data[11];
    frame_data[0] = 0x01;
    frame_data[1] = 0x10;
    frame_data[2] = 0x20;
    frame_data[3] = 0x88;
    frame_data[4] = 0x00;
    frame_data[5] = 0x02;
    frame_data[6] = 0x04;
    frame_data[7] = (left_rpm >> 8) & 0xFF;
    frame_data[8] = left_rpm & 0xFF;
    frame_data[9] = (right_rpm >> 8) & 0xFF;
    frame_data[10] = right_rpm & 0xFF;
    
    // 2. 计算CRC
    uint16_t crc_value = Calculate_CRC16(frame_data, 11);
    
    // 3. 组装完整帧（13字节）
    uint8_t final_frame[13];
    memcpy(final_frame, frame_data, 11);
    final_frame[11] = crc_value & 0xFF;          // CRC低字节在前（Modbus标准）
    final_frame[12] = (crc_value >> 8) & 0xFF;  // CRC高字节在后（Modbus标准）
    
    // 4. 发送13字节
    for (uint8_t i = 0; i < 13; i++)
    {
        USART3_SendByte(final_frame[i]);
    }
    
    // 5. 等待发送完成
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    
    // 6. Modbus帧间延时（增加到10ms，确保驱动器有足够时间响应）
    for (volatile uint32_t i = 0; i < 68000; i++);  
}

/**
  * @brief  读取左轮编码器值（32位）
  * @note   寄存器地址：0x20A7（高16位）+ 0x20A8（低16位）
  * @retval int32_t 左轮编码器累计值
  */
int32_t Modbus_ReadEncoder_Left(void)
{
    #if USE_FAKE_ENCODER_DATA
        // ========== 测试模式：使用假数据，不连接Modbus硬件 ==========
        static int32_t fake_encoder = 0;
        fake_encoder += 3;  // 模拟每次增加3个值
        
        #if LEFT_ENCODER_REVERSE
            return -fake_encoder;
        #else
            return fake_encoder;
        #endif
        // ===========================================================
    #else
        // ========== 正式模式：真实Modbus通信 ==========
        uint8_t tx_frame[8];
        uint8_t rx_buffer[9];
        
        // 1. 构建请求帧
        tx_frame[0] = 0x01;
        tx_frame[1] = 0x03;
        tx_frame[2] = 0x20;
        tx_frame[3] = 0xA7;
        tx_frame[4] = 0x00;
        tx_frame[5] = 0x02;
        
        uint16_t crc = Calculate_CRC16(tx_frame, 6);
        tx_frame[6] = crc & 0xFF;          // CRC低字节在前（Modbus标准）
        tx_frame[7] = (crc >> 8) & 0xFF;  // CRC高字节在后（Modbus标准）
        
        // 2. 清空接收缓冲区
        while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
        {
            USART_ReceiveData(USART3);
        }
        
        // 3. 发送请求
        for (uint8_t i = 0; i < 8; i++)
        {
            USART3_SendByte(tx_frame[i]);
        }
        
        // 4. 等待发送完成
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
        
        // 5. 接收响应（轮询方式，持续接收直到接收到数据，可能会超时）
        uint8_t rx_index = 0;
        uint32_t timeout = 50000;  // 粗略的超时时间
        
        while (rx_index < 9 && timeout > 0)
        {
            if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
            {
                rx_buffer[rx_index++] = USART_ReceiveData(USART3);
            }
            timeout--;
        }
        
        // 6. 超时处理
        if (timeout == 0)
        {
            for (volatile uint32_t i = 0; i < 7200; i++);
            return 0;
        }
        
        // 7. 验证CRC（小端序：低字节在前）
        uint16_t received_crc = rx_buffer[7] | (rx_buffer[8] << 8);  // CRC低字节在前（Modbus标准）
        uint16_t calculated_crc = Calculate_CRC16(rx_buffer, 7);
        
        if (received_crc != calculated_crc)
        {
            for (volatile uint32_t i = 0; i < 7200; i++);
            return 0;
        }
        
        // 8. 解析32位值
        int16_t high = (rx_buffer[3] << 8) | rx_buffer[4];
        int16_t low = (rx_buffer[5] << 8) | rx_buffer[6];
        int32_t raw_value = ((int32_t)high << 16) | (low & 0xFFFF);
        
        // 9. 帧间延时（增加到10ms，确保驱动器有足够时间响应）
        for (volatile uint32_t i = 0; i < 68000; i++);
        
        #if LEFT_ENCODER_REVERSE
            return -raw_value;
        #else
            return raw_value;
        #endif
    #endif
}

/**
  * @brief  读取右轮编码器值（32位）
  * @note   寄存器地址：0x20A9（高16位）+ 0x20AA（低16位）
  * @retval int32_t 右轮编码器累计值
  */
int32_t Modbus_ReadEncoder_Right(void)
{
    #if USE_FAKE_ENCODER_DATA
        // ========== 测试模式：使用假数据，不连接Modbus硬件 ==========
        static int32_t fake_encoder = 0;
        fake_encoder += 3;  // 模拟每次增加3个值
        
        #if RIGHT_ENCODER_REVERSE
            return -fake_encoder;  // 方向取反
        #else
            return fake_encoder;
        #endif
        // ===========================================================
    #else
        // ========== 正式模式：真实Modbus通信 ==========
        uint8_t tx_frame[8];
        uint8_t rx_buffer[9];
        
        // 1. 构建请求帧
        tx_frame[0] = 0x01;
        tx_frame[1] = 0x03;
        tx_frame[2] = 0x20;
        tx_frame[3] = 0xA9;  // 注：右轮地址
        tx_frame[4] = 0x00;
        tx_frame[5] = 0x02;
        
        uint16_t crc = Calculate_CRC16(tx_frame, 6);
        tx_frame[6] = crc & 0xFF;          // CRC低字节在前（Modbus标准）
        tx_frame[7] = (crc >> 8) & 0xFF;  // CRC高字节在后（Modbus标准）
        
        // 2. 清空接收缓冲区
        while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
        {
            USART_ReceiveData(USART3);
        }
        
        // 3. 发送请求
        for (uint8_t i = 0; i < 8; i++)
        {
            USART3_SendByte(tx_frame[i]);
        }
        
        // 4. 等待发送完成
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
        
        // 5. 接收响应
        uint8_t rx_index = 0;
        uint32_t timeout = 50000;
        
        while (rx_index < 9 && timeout > 0)
        {
            if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
            {
                rx_buffer[rx_index++] = USART_ReceiveData(USART3);
            }
            timeout--;
        }
        
        // 6. 超时处理
        if (timeout == 0)
        {
            for (volatile uint32_t i = 0; i < 7200; i++);
            return 0;
        }
        
        // 7. 验证CRC（小端序：低字节在前）
        uint16_t received_crc = rx_buffer[7] | (rx_buffer[8] << 8);  // CRC低字节在前（Modbus标准）
        uint16_t calculated_crc = Calculate_CRC16(rx_buffer, 7);
        
        if (received_crc != calculated_crc)
        {
            for (volatile uint32_t i = 0; i < 7200; i++);
            return 0;
        }
        
        // 8. 解析32位值
        int16_t high = (rx_buffer[3] << 8) | rx_buffer[4];
        int16_t low = (rx_buffer[5] << 8) | rx_buffer[6];
        int32_t raw_value = ((int32_t)high << 16) | (low & 0xFFFF);
        
        // 9. 帧间延时（增加到10ms，确保驱动器有足够时间响应）
        for (volatile uint32_t i = 0; i < 68000; i++);
        
        #if RIGHT_ENCODER_REVERSE
            return -raw_value;
        #else
            return raw_value;
        #endif
    #endif
}

/**
  * @brief  测试Modbus编码器读取功能
  * @note   通过串口2打印测试结果
  */
void Modbus_Test_ReadEncoder(void)
{
    printf("\n========== Testing Modbus Read Encoder ==========\n");
    
    // 连续读取3次
    for (int i = 0; i < 3; i++)
    {
        int32_t left = Modbus_ReadEncoder_Left();
        int32_t right = Modbus_ReadEncoder_Right();
        
        printf("Test #%d: Left=%ld, Right=%ld\n", i+1, left, right);
    }
    
    printf("=================================================\n\n");
}