/**
 * @file    servo.c
 * @brief   舵机 ×5 器件驱动（Modules/servo，S5 器件层；2026-10-05 随
 *          F407VG 迁移增补 ID5=爪旋转，帧率分组 PM10S@250Hz / SG90@50Hz）
 *
 * 《spec 对照清单》—— PWM 舵机无权威数据手册，规格依据 = Modules 规划
 * §3.2 构型表（2026-10-01 与硬件组确认）+ 实机标定：
 *  [x] 脉宽范围：500~2500µs 标称（PM10S 标称 0.5~2.5ms），0..180° 线性映射，
 *      每 id 独立标定两端（ROBOT_SERVO_PULSE_*_US_LIST）——SG90 实际行程
 *      普遍不足 180°，标定后填实测值
 *  [x] 刷新率：按组——PM10S 组 TIM9 250Hz / SG90 组 TIM5 50Hz（硬件自动
 *      发波，写 compare 即持续；两组均 1 步 = 2µs ≈ 0.18°/步，
 *      2026-10-05 F407VG 帧率分组，此前 F103 单组 50Hz）
 *  [x] 卸力：停脉冲=卸力对 PM10S / SG90 一致（2026-10-05 实测修正：
 *      PM10S 并非 fail-hold，此前"锁存保持/需 MOS 开关"表述作废）
 *  [ ] 脉宽↔角度实测标定（实机进行，验收时逐 id 填 robot_config.h）
 *  [ ] 软限位机械核对（结构定机械限位后收紧 ROBOT_SERVO_LIMIT_*_DEG_LIST）
 */
#include "servo.h"
#include "bsp_pwm.h"
#include "bsp_sys.h"
#include "robot_config.h"

/* 配置表（值来自 robot_config.h；布线/标定定案只改配置，本文件零改动） */
static const uint8_t  s_pwm_ch[SERVO_COUNT]       = ROBOT_SERVO_PWM_CH_LIST;
static const uint16_t s_pulse_min_us[SERVO_COUNT]  = ROBOT_SERVO_PULSE_MIN_US_LIST;
static const uint16_t s_pulse_max_us[SERVO_COUNT]  = ROBOT_SERVO_PULSE_MAX_US_LIST;
static const float    s_limit_min_deg[SERVO_COUNT] = ROBOT_SERVO_LIMIT_MIN_DEG_LIST;
static const float    s_limit_max_deg[SERVO_COUNT] = ROBOT_SERVO_LIMIT_MAX_DEG_LIST;

static uint32_t s_init_ms;      /* Init 时刻（中位稳定窗起点） */
static bool     s_inited;

/* 中位稳定窗：发中位脉冲后经 ROBOT_SERVO_CENTER_SETTLE_MS 方可控
 * （非阻塞——不引入 cmsis_os 依赖，Bsp_GetMs 差分计时天然抗回绕） */
static bool SettleDone(void)
{
    return s_inited &&
           (Bsp_GetMs() - s_init_ms) >= ROBOT_SERVO_CENTER_SETTLE_MS;
}

bool Servo_InitAll(void)
{
    Pwm_InitAll();      /* 幂等：只 HAL_TIM_PWM_Start，上电 compare=0（无脉冲） */
    for (uint8_t i = 0u; i < SERVO_COUNT; i++)
    {
        uint32_t center_us = ((uint32_t)s_pulse_min_us[i] + s_pulse_max_us[i]) / 2u;
        Pwm_SetPulseUs(s_pwm_ch[i], center_us);     /* 两级安全链之末级：发中位 */
    }
    s_init_ms = Bsp_GetMs();
    s_inited = true;
    return true;
}

void Servo_SetAngle(ServoId_t id, float deg)
{
    /* 忽略条件：参数非法 / 未初始化 / 中位稳定窗未过（只有舵机已稳定到
     * 中位才允许设置角度，防上电猛冲）/ NaN */
    if (id >= SERVO_COUNT || !SettleDone() || deg != deg)
    {
        return;
    }

    uint8_t i = (uint8_t)id;
    if (deg < s_limit_min_deg[i])
    {
        deg = s_limit_min_deg[i];
    }
    if (deg > s_limit_max_deg[i])
    {
        deg = s_limit_max_deg[i];
    }

    float span = (float)(s_pulse_max_us[i] - s_pulse_min_us[i]);
    uint32_t us = (uint32_t)((float)s_pulse_min_us[i] + deg * (span / 180.0f) + 0.5f);
    Pwm_SetPulseUs(s_pwm_ch[i], us);    /* 0..20000µs 物理钳位在 BSP，标定域钳位已在此完成 */
}

bool Servo_Release(ServoId_t id)
{
    if (id >= SERVO_COUNT)
    {
        return false;
    }
    Pwm_Release(s_pwm_ch[id]);      /* 停脉冲=卸力（两类器件一致） */
    return true;
}
