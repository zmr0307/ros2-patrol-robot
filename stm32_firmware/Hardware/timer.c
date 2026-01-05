#include "timer.h"
#include "stm32f10x.h"
#include "modbus.h"
#include "serial.h"
#include <stdio.h>

// ========== 修改：去掉旧的静态变量，改用全局标志 ==========
// 全局变量：50Hz定时器标志（在main.c中检测）
volatile uint8_t g_read_encoder_flag = 0;

// ============================================================

/**
  * @brief  初始化TIM2定时器为50Hz（20ms周期）
  * @note   使用STM32的TIM2，产生定时中断
  * @param  None
  * @retval None
  */
void Timer_Init(void)
{
    // 1. 使能TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 2. 配置定时器参数
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    
    // 假设系统时钟72MHz，APB1时钟36MHz，但TIM2时钟是72MHz
    // 为了得到50Hz（20ms周期）：
    // 预分频器 = 7200 - 1 = 7199 → 时钟频率变成 72MHz / 7200 = 10kHz
    // 自动重装载值 = 200 - 1 = 199 → 中断频率 = 10kHz / 200 = 50Hz
    
    TIM_TimeBaseStructure.TIM_Period = 200 - 1;        // 自动重装载值（ARR）
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;    // 预分频器（PSC）
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // 3. 使能TIM2更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    // 4. 配置NVIC（中断优先级）
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // 抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // 响应优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 5. 启动定时器
    TIM_Cmd(TIM2, ENABLE);
    
    printf("TIM2 initialized: 50Hz (20ms period)\r\n");
}

/**
  * @brief  获取当前里程计序列号
  * @retval 序列号
  * @note   这个函数现在不再使用，序列号在main.c中管理
  */
uint32_t Timer_GetOdomSeq(void)
{
    // 返回0，因为序列号已经移到main.c中
    return 0;
}

/**
  * @brief  TIM2中断处理函数（50Hz）
  * @note   每20ms执行一次，只设置标志位，不做耗时操作
  */
void TIM2_IRQHandler(void)
{
    // 检查是否是更新中断
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        // ========== 新逻辑：只设置标志位 ==========
        // 不在中断中读取编码器（避免阻塞）
        // 主循环检测到标志后再读取
        g_read_encoder_flag = 1;
        // =========================================
        
        // 清除中断标志位
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
