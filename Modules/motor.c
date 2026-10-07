/**
 * @file    motor.c
 * @brief   TB6612 ×4 器件驱动（Modules/motor，S5 开环）+ W2.3 速度环
 *          （M 法测速 + 位置式 PID，节拍由任务层 1kHz 驱动）
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
#include "bsp_encoder.h"
#include "bsp_log.h"
#include "pid.h"
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

/* ==================== W2.3 速度环 ==================== */

/* 电机↔编码器通道映射（值来自 robot_config.h，1..4 = ENC_CH1..4） */
static const uint8_t s_enc_ch[MOTOR_CH_COUNT] = {
    ROBOT_MOTOR_CH1_ENC, ROBOT_MOTOR_CH2_ENC, ROBOT_MOTOR_CH3_ENC, ROBOT_MOTOR_CH4_ENC,
};

/* 测速换算系数（rpm/count），Motor_Init 算一次：
 * rpm = (Δcount ÷ PPR) ÷ T(秒) × 60，T = CALC_MS/1000
 *   → rpm = Δcount × (60 × 1000) ÷ (PPR × CALC_MS)
 * 4096PPR、10ms 窗下 ≈ 1.46 rpm/count（1ms 直接差分 ≈ 14.6，量化过粗） */
static float s_rpm_per_count;

static PIDInstance s_speed_pid[MOTOR_CH_COUNT]; /* 每路一个实例（W2.1 PID） */
static float s_ref_rpm[MOTOR_CH_COUNT];         /* 设定转速（duty 语义符号） */
static float s_speed_rpm[MOTOR_CH_COUNT];       /* 最近一窗折算的实测转速 */
static int32_t s_win_acc[MOTOR_CH_COUNT];       /* 测速窗内编码器增量累计 */
static uint16_t s_win_tick;                     /* 窗内已累计的毫拍数 */
static bool s_speed_inited;                     /* Motor_Init 已装参（防裸调闭环） */

static void SpeedLoopInit(void)
{
    PID_Init_Config_s c;
    c.Kp = ROBOT_MOTOR_SPEED_KP;
    c.Ki = ROBOT_MOTOR_SPEED_KI;
    c.Kd = ROBOT_MOTOR_SPEED_KD;
    c.MaxOut = 1.0f;        /* 输出 = duty 满量程，直喂 Motor_SetDuty 零换算 */
    c.DeadBand = 0.0f;
    c.Improve = (PID_Improvement_e)(PID_Integral_Limit | PID_Trapezoid_Intergral |
                                    PID_Derivative_On_Measurement | PID_DerivativeFilter |
                                    PID_ErrorHandle);
    c.IntegralLimit = ROBOT_MOTOR_SPEED_INTEGRAL_LIMIT;
    c.CoefA = 0.0f;
    c.CoefB = 0.0f;
    c.Output_LPF_RC = 0.0f;
    c.Derivative_LPF_RC = ROBOT_MOTOR_SPEED_D_LPF_RC;

    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        PIDInit(&s_speed_pid[i], &c);
        s_ref_rpm[i] = 0.0f;
        s_speed_rpm[i] = 0.0f;
        s_win_acc[i] = 0;
    }
    s_win_tick = 0u;
    s_rpm_per_count = 60.0f * 1000.0f /
                      ((float)ROBOT_ENC_PPR * (float)ROBOT_MOTOR_SPEED_CALC_MS);
    s_speed_inited = true;
}

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
    SpeedLoopInit();                /* 速度环装参清态（须在 Bsp_Init 之后：PID 记时取 Bsp_GetUs） */
    return true;
}

/* 最近一次施加的 duty（含符号），调试/整定观测用 */
static float s_duty_dbg[MOTOR_CH_COUNT];

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
        s_duty_dbg[(uint8_t)ch] = 0.0f;
        return;
    }

    /* 同边沿切换：先 DIR 后 PWM。正转 = IN1=1/IN2=0（真值表 CW），
     * 轮向修正 ROBOT_MOTOR_SIGN（±1）翻转整体方向约定 */
    bool forward = ((duty > 0.0f) == (ROBOT_MOTOR_SIGN >= 0));
    (forward ? Gpio_Set : Gpio_Reset)(s_dir_a[m->dir_idx]);
    (forward ? Gpio_Reset : Gpio_Set)(s_dir_b[m->dir_idx]);
    Pwm_SetDuty(m->pwm_ch, mag);
    s_duty_dbg[(uint8_t)ch] = (forward == (ROBOT_MOTOR_SIGN >= 0)) ? mag : -mag;
}

float Motor_GetDuty(MotorCh_t ch)
{
    return (ch < MOTOR_CH_COUNT) ? s_duty_dbg[(uint8_t)ch] : 0.0f;
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
        /* 断电即清速度环意图与积分态：复能瞬间旧 ref/旧积分不会顶一下 */
        s_ref_rpm[i] = 0.0f;
        s_speed_pid[i].Iout = 0.0f;
        s_speed_pid[i].ITerm = 0.0f;
        s_speed_pid[i].Output = 0.0f;
    }
    Gpio_Reset(PIN_PWR_STBY);       /* 后写 STBY：先清 duty 后断电，次序即保险 */
}

void Motor_SetSpeedRpm(MotorCh_t ch, float rpm)
{
    if (ch >= MOTOR_CH_COUNT)
    {
        return;
    }
    if (rpm != rpm)                 /* NaN：速度指令坏数不生效，保留原 ref */
    {
        return;
    }
    if (rpm > ROBOT_MOTOR_SPEED_MAX_RPM)
    {
        rpm = ROBOT_MOTOR_SPEED_MAX_RPM;
    }
    if (rpm < -ROBOT_MOTOR_SPEED_MAX_RPM)
    {
        rpm = -ROBOT_MOTOR_SPEED_MAX_RPM;
    }
    s_ref_rpm[(uint8_t)ch] = rpm;
}

float Motor_GetSpeedRpm(MotorCh_t ch)
{
    return (ch < MOTOR_CH_COUNT) ? s_speed_rpm[(uint8_t)ch] : 0.0f;
}

void Motor_SpeedLoopUpdate(void)
{
    if (!s_speed_inited)
    {
        return;                     /* Motor_Init 未跑：不闭环不出力 */
    }

    /* ① 测速：每毫拍读一次增量累计（防 16 位计数溢出，节拍对齐控制环），
     *    CALC_MS 到点折算一次 rpm。本函数是编码器唯一读者（契约见 bsp_encoder.h），
     *    GetSpeedRpm 只读缓存不碰 Encoder_Read。
     *    符号：×ROBOT_MOTOR_SIGN 折算到 duty 语义——闭环后正 ref → 正 duty →
     *    正实测，与开环符号交叉判读"O=一致"同口径，保证负反馈 */
    s_win_tick++;
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        s_win_acc[i] += Encoder_Read(s_enc_ch[i]);
    }
    if (s_win_tick >= ROBOT_MOTOR_SPEED_CALC_MS)
    {
        for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
        {
            s_speed_rpm[i] = (float)s_win_acc[i] * s_rpm_per_count * (float)ROBOT_MOTOR_SIGN;
            s_win_acc[i] = 0;
        }
        s_win_tick = 0u;
    }

    /* ② ③ ④ 每路：PID → 堵转处置 → 出 duty */
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        float out = PIDCalculate(&s_speed_pid[i], s_speed_rpm[i], s_ref_rpm[i]);

        /* 堵转：PID 内部"指令大却没动"计数满 500 次置闩。alarm 模块（W2.5）
         * 就位后这里换成正式故障码上报；本阶段限频打印（闩本身就是约 0.5s
         * 一发的节流）后自清，闭环继续——处置口径见 pid.h"使用方清"注释 */
        if (s_speed_pid[i].ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR)
        {
            long ref10 = (long)(s_ref_rpm[i] * 10.0f);
            long spd10 = (long)(s_speed_rpm[i] * 10.0f);
            Log_Printf("[MOTOR] CH%u 堵转闩置起 ref=%ld.%ld rpm 实测=%ld.%ld rpm\r\n",
                       (unsigned)(i + 1u), ref10 / 10, ref10 % 10, spd10 / 10, spd10 % 10);
            s_speed_pid[i].ERRORHandler.ERRORType = PID_ERROR_NONE;
            s_speed_pid[i].ERRORHandler.ERRORCount = 0u;
        }

        Motor_SetDuty((MotorCh_t)i, out);
    }
}
