/**
 * @file    bsp_sys.h
 * @brief   片上系统服务：DWT 微秒计时/延时、毫秒时间戳
 *
 * 注意：HAL 时基是 TIM6（非 SysTick），毫秒一律走 HAL_GetTick。
 * 使用前必须先调 Bsp_Init()（使能 DWT 计数器），可在调度器启动前调用。
 */
#ifndef BSP_SYS_H
#define BSP_SYS_H

#include <stdint.h>

void Bsp_Init(void);
uint32_t Bsp_GetUs(void);       /* 微秒时间戳，uint32 回绕约 71.6 分钟，用于测量间隔 */
uint32_t Bsp_GetMs(void);       /* 毫秒时间戳，= HAL_GetTick() */
void Bsp_DelayUs(uint32_t us);  /* DWT 忙等，短延时专用（CSN/CE 建立时间等）；>1ms 请用 osDelay */

#endif /* BSP_SYS_H */
