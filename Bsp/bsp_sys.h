/**
 * @file    bsp_sys.h
 * @brief   片上系统服务：DWT 微秒计时/延时、毫秒时间戳
 *
 * 注意：HAL 时基是 TIM6（非 SysTick），毫秒一律走 HAL_GetTick。
 * 微秒时基是 DWT CYCCNT——本板 F407 修订 5/6 的 DWT 带 LAR 锁，Bsp_Init
 * 先解锁再使能（坑#15，详见 bsp_sys.c / docs/BSP调试日志.md）。
 * 使用前必须先调 Bsp_Init()，可在调度器启动前调用。
 */
#ifndef BSP_SYS_H
#define BSP_SYS_H

#include <stdint.h>

void Bsp_Init(void);
void Bsp_DwtReArm(void);        /* DWT 使能链重整（幂等）：调试器断开后计数冻结时调用一次即恢复 */
uint32_t Bsp_GetUs(void);       /* 微秒时间戳，uint32 回绕约 71.6 分钟，用于测量间隔 */
uint32_t Bsp_GetMs(void);       /* 毫秒时间戳，= HAL_GetTick() */
void Bsp_DelayUs(uint32_t us);  /* DWT 忙等，短延时专用（CSN/CE 建立时间等）；>1ms 请用 osDelay */

/* 芯片身份只读窥视（验芯片工具：正品 STM32F407 家族低 16 位 = 0x0413） */
uint32_t Bsp_GetDevId(void);    /* DBGMCU_IDCODE：DEV_ID[15:0] + REV_ID[31:16] */

#endif /* BSP_SYS_H */
