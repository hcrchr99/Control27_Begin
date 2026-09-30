/**
 * @file    bsp_pwm.c
 * @brief   PWM 输出实现：静态句柄/ARR 表 + 按通道所属定时器各自换算
 */
#include "bsp_pwm.h"
#include "bsp_pin.h"
#include "tim.h"

/* HAL 句柄表，组 0 = TIM4 20kHz、组 1 = TIM5 50Hz（extern 自 tim.h）。
 * ⚠ S0 教训：TIM4/TIM5 实例宏是寄存器块指针，严禁当句柄用；这里一律 &htimx。 */
static TIM_HandleTypeDef *const s_grp_tim[2] = { &htim4, &htim5 };

/* ch 1..8 → 通道 LL 宏（前 4 项属组 0，后 4 项属组 1） */
static const uint32_t s_ch_ll[8] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4,
};

/* ch 1..8 → CCR 满量程 = 所属定时器 ARR+1。占空比换算查本表，将来新增
 * PWM 组只加表项，公式不动（ARR 进 bsp_pin.h，与 CubeMX 配置对账）。 */
static const uint32_t s_ch_full[8] = {
    PIN_PWM20K_ARR + 1u, PIN_PWM20K_ARR + 1u, PIN_PWM20K_ARR + 1u, PIN_PWM20K_ARR + 1u,
    PIN_PWM50HZ_ARR + 1u, PIN_PWM50HZ_ARR + 1u, PIN_PWM50HZ_ARR + 1u, PIN_PWM50HZ_ARR + 1u,
};

void Pwm_InitAll(void)
{
    for (uint32_t ch = PWM_20K_CH1; ch <= PWM_50HZ_CH4; ch++)
    {
        (void)HAL_TIM_PWM_Start(s_grp_tim[ch > PWM_20K_CH4], s_ch_ll[ch - 1u]);
    }
}

void Pwm_SetDuty(uint8_t ch, float duty)
{
    if (ch < PWM_20K_CH1 || ch > PWM_50HZ_CH4)
    {
        return;
    }
    uint32_t idx = ch - PWM_20K_CH1;

    if (!(duty > 0.0f))     /* 负值 / 0 / NaN（比较恒 false）统一落安全态 */
    {
        __HAL_TIM_SET_COMPARE(s_grp_tim[ch > PWM_20K_CH4], s_ch_ll[idx], 0u);
        return;
    }
    if (duty > 1.0f)
    {
        duty = 1.0f;
    }
    __HAL_TIM_SET_COMPARE(s_grp_tim[ch > PWM_20K_CH4], s_ch_ll[idx],
                          (uint32_t)(duty * (float)s_ch_full[idx] + 0.5f));
}

void Pwm_SetPulseUs(uint8_t ch, uint32_t us)
{
    if (ch < PWM_50HZ_CH1 || ch > PWM_50HZ_CH4)
    {
        return;
    }

    uint32_t steps = us / PIN_PWM50HZ_US_PER_STEP;
    if (steps > PIN_PWM50HZ_ARR + 1u)   /* 0..20000us（0..1000 步），超界钳满 */
    {
        steps = PIN_PWM50HZ_ARR + 1u;
    }
    __HAL_TIM_SET_COMPARE(&htim5, s_ch_ll[ch - 1u], steps);
}

void Pwm_Release(uint8_t ch)
{
    if (ch < PWM_20K_CH1 || ch > PWM_50HZ_CH4)
    {
        return;
    }
    __HAL_TIM_SET_COMPARE(s_grp_tim[ch > PWM_20K_CH4], s_ch_ll[ch - 1u], 0u);
}
