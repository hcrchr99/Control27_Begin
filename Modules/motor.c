/**
 * @file    motor.c
 * @brief   TB6612 ×4 器件驱动（Modules/motor，S5 开环）
 *
 * 《spec 对照清单》—— TB6612FNG（东芝数据手册，控制真值表逐条核对，
 * 2026-10-02 查证；验收时逐条打勾）：
 *  [x] 控制真值表（STBY=H 时）：IN1=H/IN2=L/PWM=H → CW；IN1=L/IN2=H/PWM=H →
 *      CCW；IN1=IN2=H → 短刹；IN1=IN2=L → STOP（输出高阻，滑行）；
 *      PWM=L 且方向有效 → 短刹（PWM 斩波 = 驱动/短刹交替，占空比即平均电压）
 *  [x] STBY=L → 待机：优先级最高，输出关断，与 IN1/IN2/PWM 无关
 *      —— Disable 双保险、上电安全链的物理依据
 *  [x] STBY 上电态：CubeMX 复位电平 = 低（gpio.c 输出电平 RESET 已核实），
 *      bsp_gpio 复位保持 = 安全链第 0 级不依赖上层调用顺序
 *  [x] PWM 输入频率：20kHz 为 S4 已实机验收的 TIM4 输出，驱动斩波正常
 *      （手册 fPWM 上限值随 VM 条件标注，远高于 20kHz，实机运行即证据）
 *  [x] 逻辑电平：VCC 2.7~5.5V，本工程 3.3V 直连 STM32 兼容
 *  [ ] 方向基准：CW/CCW 与整车"前进"的对应 = 轮向修正项，开环+编码器符号
 *      交叉验证后定 ROBOT_MOTOR_SIGN 与通道映射（robot_config.h）
 *
 * 实现要点（Modules 规划 §3.1）：① 符号→方向映射在本层（BSP 的 PWM 无方向
 * 概念）；② 同边沿切换：改向时先置新 DIR 再写 PWM，杜绝共导通；③ 死区
 * ROBOT_MOTOR_DEADBAND 占位；④ 通道→(PWM ch, DIR 引脚对) 映射表收
 * robot_config.h 一处改，本文件代码零改动；⑤ Disable = duty 0 + STBY 低。
 */
#include "motor.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"
#include "robot_config.h"

/* 通道映射表（值来自 robot_config.h；DIR 序号 1..4 → 引脚对 PIN_DIRxA/B） */
typedef struct
{
    uint8_t pwm_ch;     /* bsp_pwm 20kHz 通道号（PWM_20K_CH1..4） */
    uint8_t dir_idx;    /* DIR 引脚对序号 0..3 */
} MotorMap_t;

static const GpioPin_t s_dir_a[MOTOR_CH_COUNT] = { PIN_DIR1A, PIN_DIR2A, PIN_DIR3A, PIN_DIR4A };
static const GpioPin_t s_dir_b[MOTOR_CH_COUNT] = { PIN_DIR1B, PIN_DIR2B, PIN_DIR3B, PIN_DIR4B };

static const MotorMap_t s_map[MOTOR_CH_COUNT] = {
    { ROBOT_MOTOR_CH1_PWM, ROBOT_MOTOR_CH1_DIR - 1 },
    { ROBOT_MOTOR_CH2_PWM, ROBOT_MOTOR_CH2_DIR - 1 },
    { ROBOT_MOTOR_CH3_PWM, ROBOT_MOTOR_CH3_DIR - 1 },
    { ROBOT_MOTOR_CH4_PWM, ROBOT_MOTOR_CH4_DIR - 1 },
};

/* IN=00（STOP 滑行）：duty 0 与 Disable 共用的关断写法 */
static void CoastChannel(const MotorMap_t *m)
{
    Gpio_Reset(s_dir_a[m->dir_idx]);
    Gpio_Reset(s_dir_b[m->dir_idx]);
    Pwm_SetDuty(m->pwm_ch, 0.0f);
}

bool Motor_Init(void)
{
    Pwm_InitAll();                  /* 幂等：只 HAL_TIM_PWM_Start，保持上电 compare=0 */
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        CoastChannel(&s_map[i]);
    }
    Gpio_Reset(PIN_PWR_STBY);       /* 保持关断（CubeMX 复位已低，此处幂等） */
    return true;
}

void Motor_SetDuty(MotorCh_t ch, float duty)
{
    if (ch >= MOTOR_CH_COUNT)
    {
        return;
    }

    float mag = (duty < 0.0f) ? -duty : duty;
    if (mag > 1.0f)
    {
        mag = 1.0f;
    }
    const MotorMap_t *m = &s_map[(uint8_t)ch];
    if (duty != duty || mag < ROBOT_MOTOR_DEADBAND)     /* NaN 失安全 → 滑行 */
    {
        CoastChannel(m);            /* IN=00 → 滑行，与符号无关 */
        return;
    }

    /* 同边沿切换：先 DIR 后 PWM。正转 = IN1=1/IN2=0（真值表 CW），
     * 轮向修正 ROBOT_MOTOR_SIGN（±1）翻转整体方向约定 */
    bool forward = ((duty > 0.0f) == (ROBOT_MOTOR_SIGN >= 0));
    (forward ? Gpio_Set : Gpio_Reset)(s_dir_a[m->dir_idx]);
    (forward ? Gpio_Reset : Gpio_Set)(s_dir_b[m->dir_idx]);
    Pwm_SetDuty(m->pwm_ch, mag);
}

void Motor_Enable(void)
{
    Gpio_Set(PIN_PWR_STBY);
}

void Motor_Disable(void)
{
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        CoastChannel(&s_map[i]);
    }
    Gpio_Reset(PIN_PWR_STBY);       /* 后写 STBY：先清 duty 后断电，次序即保险 */
}
