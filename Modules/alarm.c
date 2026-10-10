/**
 * @file    alarm.c
 * @brief   声光告警实现：音型表查表 + 绝对时间推进相位（见 alarm.h 表格）
 *
 * 实现：seq[] 交替存"响/停"时长（偶下标=响），Poll 里到点翻相位、每相位
 * 一次写齐蜂鸣器与 LED1——不依赖调用节拍，长时间不调也不追帧（下次调用
 * 从当前相位继续，最多慢一拍，告警场景可接受）。
 * 蜂鸣器为无源件：响=PWM_TIM14_CH1 50% duty（方波激励，音调由 TIM14 时基
 * 定），停=duty 0（与 Pwm_Release 同效，compare=0 恒低静音）。
 */
#include "alarm.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "bsp_sys.h"

/* 音型表（ms；与 alarm.h 注释表同步维护，追加码三处同步：枚举/本表/越界上界） */
static const uint16_t SEQ_LINK_LOST[]    = { 200, 200, 200, 1400 };
static const uint16_t SEQ_POWER_LOW[]    = { 800, 800 };
static const uint16_t SEQ_TASK_OVERRUN[] = { 100, 100, 100, 100, 100, 1500 };
static const uint16_t SEQ_ACT_STALL[]    = { 600, 200, 100, 100, 100, 2000 };

typedef struct
{
    const uint16_t *seq;    /* 交替 响/停 时长，偶下标 = 响 */
    uint8_t         n;      /* seq 元素数 */
} AlarmPattern_t;

/* 下标 = AlarmCode_t（[0]=OK 占位不用），越界判据取自本表长度 */
static const AlarmPattern_t s_pat[] = {
    { (const uint16_t *)0, 0u },                                    /* OK */
    { SEQ_LINK_LOST,    4u },                                       /* LINK_LOST */
    { SEQ_POWER_LOW,    2u },                                       /* POWER_LOW */
    { SEQ_TASK_OVERRUN, 6u },                                       /* TASK_OVERRUN */
    { SEQ_ACT_STALL,    6u },                                       /* ACT_STALL */
};

#define ALARM_CODE_MAX  ((AlarmCode_t)((sizeof(s_pat) / sizeof(s_pat[0])) - 1u))

static volatile AlarmCode_t s_code = ALARM_OK;  /* 业务侧写入，仅单字节赋值 */
static AlarmCode_t s_played = ALARM_OK;         /* 正在播放的码（检测切换用） */
static uint8_t  s_phase;                        /* 当前相位下标（偶=响） */
static uint32_t s_phase_t0;                     /* 本相位起始时刻（ms） */

void Alarm_Set(AlarmCode_t code)
{
    if ((uint32_t)code > (uint32_t)ALARM_CODE_MAX)
    {
        return;                                 /* 超界忽略（未知码不发声光） */
    }
    s_code = code;
}

AlarmCode_t Alarm_Get(void)
{
    return s_code;
}

/* 把一个相位落到声光输出口：偶相位=响+亮，奇相位=停+灭；
 * OK 码只静音、不碰 LED1（心跳域，见 alarm.h）。
 * 蜂鸣器无源件：响=50% duty 方波（TIM14 时基定音调），停=0 duty 静音 */
static void Apply(AlarmCode_t code, uint8_t phase)
{
    if (code == ALARM_OK)
    {
        Pwm_SetDuty(PWM_TIM14_CH1, 0.0f);
        return;
    }
    bool on = ((phase & 1u) == 0u);
    Pwm_SetDuty(PWM_TIM14_CH1, on ? 0.5f : 0.0f);
    (on ? Gpio_Set : Gpio_Reset)(PIN_LED1);
}

void Alarm_Poll(void)
{
    uint32_t now = Bsp_GetMs();

    if (s_code != s_played)
    {   /* 码切换：相位归零，立即呈现第一相 */
        s_played = s_code;
        s_phase = 0u;
        s_phase_t0 = now;
        Apply(s_played, s_phase);
        return;
    }
    if (s_played == ALARM_OK)
    {
        return;                                 /* 静音态无事可做 */
    }

    const AlarmPattern_t *p = &s_pat[s_played];
    if ((now - s_phase_t0) >= p->seq[s_phase])
    {
        s_phase = (uint8_t)((s_phase + 1u) % p->n);
        s_phase_t0 = now;
        Apply(s_played, s_phase);
    }
}
