#include "stm32f10x.h"
#include "serial.h"
#include <stdio.h> // <-- NEW: Include for FILE type needed by fputc

//========== State Machine (No changes) ==========
typedef enum
{
    STATE_WAIT_HEADER,
    STATE_RECEIVE_DATA
} RxState;

static volatile RxState currentState = STATE_WAIT_HEADER;

//========== Receiver Variables (No changes) ==========
static uint8_t rx_buffer[10];
static volatile uint8_t rx_index = 0;
static volatile uint8_t rx_complete = 0;

// Forward declaration for the new debug serial init function
void Debug_Serial_Init(void); // <-- NEW

//========================================================================
// Serial Initialization (Heavily Modified)
//========================================================================
void Serial_Init(void)
{
    // --- Initialize USART1 (PA9, PA10) for Robot Control ---
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    // USART1 TX (PA9) - We are not sending from USART1 to the robot, but init it anyway
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART1 RX (PA10) - This is where data from the blue module comes in
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure); // Speed is inherited

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // <-- MODIFIED: USART1 now only needs to receive. We set Tx as well for completeness.
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

    // Interrupt setup for USART1 remains the same
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);

    // --- Initialize USART2 (PA2, PA3) for Debugging (printf) ---
    Debug_Serial_Init(); // <-- NEW: Call the init function for the debug port
}

//========================================================================
// NEW FUNCTION: Initialize USART2 for Debugging to PC
//========================================================================
void Debug_Serial_Init(void)
{
    // Enable clocks for USART2 and its GPIO pins (Port A)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    // RCC_APB2Periph_GPIOA is already enabled above

    GPIO_InitTypeDef GPIO_InitStructure;
    // USART2 TX (PA2) - This sends debug info to the PC
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // USART2 RX (PA3) - We are not receiving from the PC, but init it as floating input
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // <-- MODIFIED: USART2's main purpose is to Transmit (Tx).
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
}

//========================================================================
// Send Functions (Modified to use USART2)
//========================================================================
void Serial_SendByte(uint8_t Byte)
{
    // <-- MODIFIED: This function now sends data through USART2
    USART_SendData(USART2, Byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        ;
}

void Serial_SendString(char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        Serial_SendByte(String[i]);
    }
}

// This function now redirects printf to USART2 automatically
int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}

//========================================================================
// Receive Functions (No changes needed, they are for USART1)
//========================================================================
uint8_t Serial_GetRxFlag(void)
{
    if (rx_complete)
    {
        rx_complete = 0;
        return 1;
    }
    return 0;
}

uint8_t *Serial_GetRxBuffer(void)
{
    return rx_buffer;
}

//========================================================================
// USART1 Interrupt Handler (No changes needed, it handles control data)
//========================================================================
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t data = USART_ReceiveData(USART1);

        switch (currentState)
        {
        case STATE_WAIT_HEADER:
        {
            if (data == 0xA5)
            {
                rx_buffer[0] = data;
                rx_index = 1;
                currentState = STATE_RECEIVE_DATA;
            }
            break;
        }

        case STATE_RECEIVE_DATA:
        {
            rx_buffer[rx_index] = data;
            rx_index++;

            if (rx_index >= 10)
            {
                rx_complete = 1;
                rx_index = 0;
                currentState = STATE_WAIT_HEADER;
            }
            break;
        }
        }

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

/**
 * @brief  发送里程计数据帧（10字节）
 * @param  seq: 序列号
 * @param  delta_left: 左轮编码器增量
 * @param  delta_right: 右轮编码器增量
 * @note   通过USART2发送给Jetson
 */
void Serial_SendOdomFrame(uint32_t seq, int16_t delta_left, int16_t delta_right)
{
    uint8_t frame[10];
    uint8_t checksum = 0;

    // 1. 帧头
    frame[0] = 0xA5;

    // 2. 序列号（大端序，高字节在前）
    frame[1] = (seq >> 24) & 0xFF;
    frame[2] = (seq >> 16) & 0xFF;
    frame[3] = (seq >> 8) & 0xFF;
    frame[4] = seq & 0xFF;

    // 3. 左轮增量（大端序）
    frame[5] = (delta_left >> 8) & 0xFF;
    frame[6] = delta_left & 0xFF;

    // 4. 右轮增量（大端序）
    frame[7] = (delta_right >> 8) & 0xFF;
    frame[8] = delta_right & 0xFF;

    // 5. 计算校验和（字节1-8的累加和）
    for (uint8_t i = 1; i < 9; i++)
    {
        checksum += frame[i];
    }
    frame[9] = checksum;

    // 6. 发送10字节
    for (uint8_t i = 0; i < 10; i++)
    {
        Serial_SendByte(frame[i]);
    }
}

/**
 * @brief  测试里程计数据发送功能
 * @note   发送3帧测试数据，观察格式是否正确
 */
void Serial_Test_SendOdom(void)
{
    printf("\n========== Testing Odom Frame Send ==========\n");
    printf("Sending 3 test frames...\n\n");

    // 测试帧1：序列号1，左右增量都是14
    printf("Frame #1: seq=1, delta_L=14, delta_R=14\n");
    printf("  Expected: A5 00 00 00 01 00 0E 00 0E 1C\n");
    printf("  Sending:  ");
    Serial_SendOdomFrame(1, 14, 14);
    printf("\n\n");

    // 延时，便于观察
    for (volatile uint32_t i = 0; i < 1000000; i++)
        ;

    // 测试帧2：序列号2，左右增量都是28
    printf("Frame #2: seq=2, delta_L=28, delta_R=28\n");
    printf("  Expected: A5 00 00 00 02 00 1C 00 1C 3C\n");
    printf("  Sending:  ");
    Serial_SendOdomFrame(2, 28, 28);
    printf("\n\n");

    for (volatile uint32_t i = 0; i < 1000000; i++)
        ;

    // 测试帧3：序列号3，左增量14，右增量-14（测试负数）
    printf("Frame #3: seq=3, delta_L=14, delta_R=-14\n");
    printf("  Expected: A5 00 00 00 03 00 0E FF F2 06\n");
    printf("  Sending:  ");
    Serial_SendOdomFrame(3, 14, -14);
    printf("\n\n");

    printf("==============================================\n\n");
}
