/**
 * @file    bsp_sys_stub.h
 * @brief   Bsp_GetUs/Ms 的 host 可控替身（仅 PC 单测）
 *
 * pid.c 依赖 bsp_sys.h（仅函数原型，无 HAL），host 测试用可控假时钟
 * 替换真 DWT——用例通过 Set/Advance 精确控制 PID 的 dt。
 */
#ifndef BSP_SYS_STUB_H
#define BSP_SYS_STUB_H

#include <stdint.h>

void Stub_Time_SetUs(uint32_t us);       /* 时钟直接置位 */
void Stub_Time_AdvanceUs(uint32_t d_us); /* 时钟前进 d_us 微秒 */
uint32_t Stub_Time_NowUs(void);          /* 读当前假时钟 */

#endif /* BSP_SYS_STUB_H */
