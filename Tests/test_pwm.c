/**
 * @file    test_pwm.c
 * @brief   S4 测试项：TIM4 20kHz 占空比阶梯 + TIM5 50Hz 脉宽阶梯（示波器验收）
 *
 * 验收标准（BSP层开发规划 六.S4）：20kHz duty / 50Hz 脉宽输出经示波器验证。
 *  - 20K 组 PB6~PB9 四路同步：0→25%→50%→75%→100% 循环，每档 2s，
 *    示波器读频率 20kHz、占空比逐档爬升（100% 档应恒高无边沿）；
 *  - 50Hz 组 PA0~PA3 四路同步：500→1000→1500→2000→2500us 循环，每档 2s，
 *    示波器读频率 50Hz、正脉宽逐档爬升（20000us 恒高档不在此序列，
 *    超界钳位逻辑靠代码审查：SetPulseUs>20000→20000、SetDuty<0→0）。
 */
#include "test_bench.h"
#include "bsp_pwm.h"
#include "bsp_sys.h"
#include "bsp_log.h"

void Test_Pwm_Init(void)
{
    Pwm_InitAll();
    Log_Printf("[T-PWM] 20K(PB6-9) duty 阶梯 / 50Hz(PA0-3) 脉宽阶梯，每档 2s，上示波器\r\n");
}

void Test_Pwm_Poll(void)
{
    static uint32_t s_last;
    static uint8_t  s_step;
    uint32_t now = Bsp_GetMs();
    if ((now - s_last) < 2000u)
    {
        return;
    }
    s_last = now;

    static const float    s_duty[] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
    static const uint32_t s_us[]   = { 500u, 1000u, 1500u, 2000u, 2500u };
    const uint8_t n = (uint8_t)(sizeof(s_duty) / sizeof(s_duty[0]));
    uint8_t i = s_step % n;
    s_step++;

    for (uint8_t ch = PWM_20K_CH1; ch <= PWM_20K_CH4; ch++)
    {
        Pwm_SetDuty(ch, s_duty[i]);
    }
    for (uint8_t ch = PWM_50HZ_CH1; ch <= PWM_50HZ_CH4; ch++)
    {
        Pwm_SetPulseUs(ch, s_us[i]);
    }

    Log_Printf("[T-PWM] 20K duty=%u%% | 50Hz width=%uus\r\n",
               (unsigned)(s_duty[i] * 100.0f), (unsigned)s_us[i]);
}
