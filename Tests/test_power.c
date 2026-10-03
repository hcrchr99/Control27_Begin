/**
 * @file    test_power.c
 * @brief   S6 测试项 8=POWER：V/I/P 打印对表 + 双 ADC 同步性 + ADC3 双通道
 *
 * 验收标准（BSP 规划 §六 / Modules 规划 §3.4）：
 *  - V/I 串口打印与万用表误差 <5%（T3，系数对表定值进 robot_config.h）；
 *  - 双 ADC 同步性：负载突变时 U/I 同拍变化（T4，同打印行内 I↑ 且 V↓）；
 *  - ADC3 关节电流两路可读、不串扰（T5）。
 *
 * 操作（板上）：
 *  - 常态 500ms 打印 raw 与换算值（newlib-nano 未启 _printf_float，
 *    统一 mV/mA/mW 整型打印；NaN 打印 -1）；
 *  - KEY1 = 负载阶跃（复用 S5 已验收 motor：MOTOR_CH1 duty 0↔0.5，
 *    消抖后翻转，供 T3 电流对表与 T4 同步性观察；不按键时电机安全断开）；
 *  - T0/T1 字序与循环存活验证：PA4/PA5 接已知电平看 raw 对应关系。
 */
#include "test_bench.h"
#include "power.h"
#include "motor.h"
#include "bsp_adc.h"
#include "bsp_pin.h"
#include "bsp_gpio.h"
#include "bsp_sys.h"
#include "bsp_log.h"

/* 负载阶跃占空比（开环诊断值，正反转由 MOTOR_SIGN 占位决定，仅作负载用） */
#define TEST_POWER_LOAD_DUTY    0.5f
/* 按键消抖：连续 N 拍(10ms)电平一致才认稳定 */
#define TEST_POWER_KEY_STABLE   3u

/* float(物理量) → 整型打印值（×1000，即 mV/mA/mW）；NaN → -1 */
static int32_t ToMilli(float x)
{
    return (x != x) ? -1 : (int32_t)(x * 1000.0f);
}

/* KEY1 稳定下降沿检测（消抖后翻转，返回 true 一次） */
static bool Key1_PressedEdge(void)
{
    static uint8_t s_cnt;
    static bool    s_stable = true;     /* 上拉空闲 = 高 */

    bool level = Gpio_Read(PIN_KEY1);
    if (level == s_stable)
    {
        s_cnt = 0u;
        return false;
    }
    if (++s_cnt >= TEST_POWER_KEY_STABLE)
    {
        s_cnt = 0u;
        s_stable = level;
        return !level;                  /* 稳定到低 = 按下沿 */
    }
    return false;
}

void Test_Power_Init(void)
{
    bool ok = Power_Init();
    Log_Printf("[T-POWER] 8=POWER %s：500ms 打印 V/I/P+J0/J1，KEY1=负载阶跃"
               "（%s 缺省半字=ADC1/IN4 电流）\r\n",
               ok ? "就绪" : "失败",
               ok ? "raw_i=低16" : "检查bsp日志；预期");
    Log_Printf("[T-POWER] 标定系数 VK=%dm IK=%dm JK=%dm（×0.001，待对表定值）\r\n",
               (int)(ROBOT_POWER_V_K * 1000.0f),
               (int)(ROBOT_POWER_I_K * 1000.0f),
               (int)(ROBOT_POWER_JOINT_I_K * 1000.0f));
}

void Test_Power_Poll(void)
{
    static uint32_t s_last;
    uint32_t now = Bsp_GetMs();

    /* KEY1 负载阶跃：采样必须在 10ms Poll 节拍（打印门之前）——若放进下方
     * 500ms 打印门内，消抖实际变成 3×500ms≈1.5s 长按才响应（S6 板上实测） */
    static bool s_load_on;
    if (Key1_PressedEdge())
    {
        s_load_on = !s_load_on;
        if (s_load_on)
        {
            Motor_Enable();
            Motor_SetDuty(MOTOR_CH1, TEST_POWER_LOAD_DUTY);
        }
        else
        {
            Motor_SetDuty(MOTOR_CH1, 0.0f);
            Motor_Disable();
        }
        Log_Printf("[T-POWER] 负载阶跃 → %s\r\n", s_load_on ? "ON" : "OFF");
    }

    if ((now - s_last) < 500u)
    {
        return;
    }
    s_last = now;

    AdcPair_t latest = { 0u, 0u };
    (void)Adc_GetLatest(&latest);       /* 失败时保持 0，行内以换算值 -1 判读 */
    int32_t v_mV = ToMilli(Power_GetVoltage());
    int32_t i_mA = ToMilli(Power_GetCurrent());
    int32_t p_mW = ToMilli(Power_GetPower());
    int32_t j0_mA = ToMilli(Power_GetJointCurrent(0));
    int32_t j1_mA = ToMilli(Power_GetJointCurrent(1));

    Log_Printf("[T-POWER] raw i=%4u v=%4u | V=%dmV I=%dmA P=%dmW |"
               " J0=%dmA J1=%dmA | load=%s\r\n",
               latest.raw_i, latest.raw_v,
               v_mV, i_mA, p_mW, j0_mA, j1_mA,
               s_load_on ? "ON" : "OFF");
}
