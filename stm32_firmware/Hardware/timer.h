#ifndef __TIMER_H
#define __TIMER_H

#include <stdint.h>

/**
  * @brief  初始化TIM2定时器，配置为50Hz（20ms周期）
  * @note   用于定时读取编码器并发送里程计数据
  * @param  None
  * @retval None
  */
void Timer_Init(void);

/**
  * @brief  获取当前里程计序列号
  * @note   每次定时器中断时序列号自动递增
  * @retval 当前序列号
  */
uint32_t Timer_GetOdomSeq(void);

// ========== 新增：全局标志声明 ==========
/**
  * @brief  50Hz定时器标志
  * @note   定时器中断时设置为1，主循环检测并清零
  */
extern volatile uint8_t g_read_encoder_flag;
// ========================================

#endif
