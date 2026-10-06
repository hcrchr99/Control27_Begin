/**
 * @file    bsp_pwm.c
 * @brief   PWM 输出实现：静态句柄/ARR/组别表 + 按通道所属定时器各自换算
 */
#include "bsp_pwm.h"
#include "bsp_pin.h"
#include "tim.h"

/* HAL 句柄表，组 0 = TIM4 20kHz、组 1 = TIM9 250Hz、组 2 = TIM5 50Hz
 * （extern 自 tim.h）。⚠ S0 教训：TIM4/TIM9/TIM5 实例宏是寄存器块指针，
 * 严禁当句柄用；这里一律 &htimx。 */
static TIM_HandleTypeDef *const s_grp_tim[3] = { &htim4, &htim9, &htim5 };

/* ch 1..9 → 通道 LL 宏（前 4 项属组 0，中 2 项属组 1，后 3 项属组 2） */
static const uint32_t s_ch_ll[9] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
    TIM_CHANNEL_1, TIM_CHANNEL_2,
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
};

/* ch 1..9 → 所属组下标（Pwm_InitAll/SetDuty/Release 按它取句柄；
 * >0 即脉宽组，Pwm_SetPulseUs 只放行这两段） */
static const uint8_t s_ch_grp[9] = { 0, 0, 0, 0, 1, 1, 2, 2, 2 };

/* ch 1..9 → CCR 满量程 = 所属定时器 ARR+1。占空比换算查本表，将来新增
 * PWM 组只加表项，公式不动（ARR 进 bsp_pin.h，与 CubeMX 配置对账）。 */
static const uint32_t s_ch_full[9] = {
    PIN_PWM20K_ARR + 1u, PIN_PWM20K_ARR + 1u, PIN_PWM20K_ARR + 1u, PIN_PWM20K_ARR + 1u,
    PIN_PWM250HZ_ARR + 1u, PIN_PWM250HZ_ARR + 1u,
    PIN_PWM50HZ_ARR + 1u, PIN_PWM50HZ_ARR + 1u, PIN_PWM50HZ_ARR + 1u,
};

/* 脉宽换算步距（µs/步）：250Hz 组与 50Hz 组当前同为 2µs，仍分表记账——
 * 两组时基独立，将来任一组改 PSC 只动自己的常量 */
static const uint8_t s_grp_us_per_step[3] = {
    0u,                      /* 组 0：20kHz 占空比组，无脉宽语义 */
    PIN_PWM250HZ_US_PER_STEP,
    PIN_PWM50HZ_US_PER_STEP,
};

void Pwm_InitAll(void)
{
    for (uint32_t ch = PWM_20K_CH1; ch <= PWM_50HZ_CH3; ch++)
    {
        (void)HAL_TIM_PWM_Start(s_grp_tim[s_ch_grp[ch - 1u]], s_ch_ll[ch - 1u]);
    }
}

void Pwm_SetDuty(uint8_t ch, float duty)
{
    if (ch < PWM_20K_CH1 || ch > PWM_50HZ_CH3)
    {
        return;
    }
    uint32_t idx = ch - PWM_20K_CH1;

    if (!(duty > 0.0f))     /* 负值 / 0 / NaN（比较恒 false）统一落安全态 */
    {
        __HAL_TIM_SET_COMPARE(s_grp_tim[s_ch_grp[idx]], s_ch_ll[idx], 0u);
        return;
    }
    if (duty > 1.0f)
    {
        duty = 1.0f;
    }
    __HAL_TIM_SET_COMPARE(s_grp_tim[s_ch_grp[idx]], s_ch_ll[idx],
                          (uint32_t)(duty * (float)s_ch_full[idx] + 0.5f));
}

void Pwm_SetPulseUs(uint8_t ch, uint32_t us)
{
    if (ch < PWM_250HZ_CH1 || ch > PWM_50HZ_CH3)
    {
        return;
    }
    uint32_t idx = ch - PWM_20K_CH1;
    uint8_t grp = s_ch_grp[idx];

    uint32_t steps = us / s_grp_us_per_step[grp];
    if (steps > s_ch_full[idx])     /* 超帧长钳到恒高（250Hz 组 4000µs / 50Hz 组 20000µs） */
    {
        steps = s_ch_full[idx];
    }
    __HAL_TIM_SET_COMPARE(s_grp_tim[grp], s_ch_ll[idx], steps);
}

void Pwm_Release(uint8_t ch)
{
    if (ch < PWM_20K_CH1 || ch > PWM_50HZ_CH3)
    {
        return;
    }
    __HAL_TIM_SET_COMPARE(s_grp_tim[s_ch_grp[ch - 1u]], s_ch_ll[ch - 1u], 0u);
}
