/**
 * @file    bsp_sys_stub.c
 * @brief   bsp_sys 的 host 替身实现（仅 PC 单测；固件用真 Bsp/bsp_sys.c）
 */
#include "bsp_sys_stub.h"
#include "bsp_sys.h"

static uint32_t s_us;

void Stub_Time_SetUs(uint32_t us)
{
    s_us = us;
}

void Stub_Time_AdvanceUs(uint32_t d_us)
{
    s_us += d_us;
}

uint32_t Stub_Time_NowUs(void)
{
    return s_us;
}

/* —— bsp_sys.h 声明的正式接口，host 侧全部落到假时钟上 —— */

void Bsp_Init(void)
{
}

uint32_t Bsp_GetUs(void)
{
    return s_us;
}

uint32_t Bsp_GetMs(void)
{
    return s_us / 1000u;
}

void Bsp_DelayUs(uint32_t us)
{
    (void)us;
}
