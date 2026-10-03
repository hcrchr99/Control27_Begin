/**
 * @file    power.c
 * @brief   功率采样换算实现
 *
 * 换算公式（线性）：physical = raw/4095 × VREF × K。
 * VREF 取 VDDA 标称 3.3V——实测偏差折叠进 K（唯一标定入口 robot_config.h），
 * 不在本模块出现第二个可调常数。
 */
#include "power.h"
#include "bsp_adc.h"
#include "robot_config.h"
#include <math.h>

/* ADC 满量程：VREF 与原始码上限（F1 右对齐 12 位） */
#define POWER_VREF      3.3f
#define POWER_RAW_MAX   4095.0f

static bool s_ready = false;

bool Power_Init(void)
{
    Adc_Init();                     /* 幂等：main.c 已启过则此调用无副作用 */
    s_ready = Adc_IsReady();
    return s_ready;
}

float Power_GetVoltage(void)
{
    AdcPair_t avg;
    if (!s_ready || !Adc_GetAvg(&avg))
    {
        return NAN;
    }
    return (avg.raw_v / POWER_RAW_MAX) * POWER_VREF * ROBOT_POWER_V_K;
}

float Power_GetCurrent(void)
{
    AdcPair_t avg;
    if (!s_ready || !Adc_GetAvg(&avg))
    {
        return NAN;
    }
    return (avg.raw_i / POWER_RAW_MAX) * POWER_VREF * ROBOT_POWER_I_K;
}

float Power_GetPower(void)
{
    AdcPair_t avg;
    if (!s_ready || !Adc_GetAvg(&avg))
    {
        return NAN;
    }
    /* 同窗快照相乘：U/I 来自同一次 GetAvg，与双同步硬件无相位差叠加 */
    float voltage = (avg.raw_v / POWER_RAW_MAX) * POWER_VREF * ROBOT_POWER_V_K;
    float current = (avg.raw_i / POWER_RAW_MAX) * POWER_VREF * ROBOT_POWER_I_K;
    return voltage * current;
}

float Power_GetJointCurrent(uint8_t k)
{
    if (!s_ready || k >= ROBOT_POWER_JOINT_COUNT)
    {
        return NAN;
    }
    uint16_t raw;
    /* k=0/1 → ADC3_CH1/CH2（IN12/IN13，映射关系见 bsp_pin.h 唯一硬件地图） */
    if (!Adc3_Read((uint8_t)(k + 1u), &raw))
    {
        return NAN;
    }
    return (raw / POWER_RAW_MAX) * POWER_VREF * ROBOT_POWER_JOINT_I_K;
}
