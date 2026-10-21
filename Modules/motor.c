/**
 * @file    motor.c
 * @brief   TB6612 ×4 器件驱动（Modules/motor，S5 开环）+ W2.3 速度环
 *          （M/T 测速 + 位置式 PID，节拍由任务层 1kHz 驱动；
 *          2026-10-21 测速自 M 法 20ms 窗换血为 M/T，见 mt-speed-pending）
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
#include "bsp_sys.h"
#include "bsp_log.h"
#include "pid.h"
#include "speed_est.h"
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

/* M/T 测速（mt-speed-pending S-C）：每拍 TakeNew 取采样环新样本 →
 * SpeedEst 边沿吸附窗折算，1kHz 全程新鲜。旧 M 法 20ms 窗退役（低速一格
 * 台阶 1.2rpm，是麦轮低速横移"一下一下"案的主根因之一）；"折算一律按
 * 实测时间"的老教训由 Δt 全实测继承。
 * ⚠ 本模块是采样环的唯一常驻消费者（Encoder_TakeNew 游标在 bsp 内）；
 * Encoder_Read 直读仍在，属旁路观察、互不干扰，但闭环读数只认 M/T */
static SpeedEst_t s_speed_est[MOTOR_CH_COUNT];  /* 每路一个 M/T 估计器 */
static SpeedEstDrip_t s_se_drip[MOTOR_CH_COUNT];    /* 滴喂历史：1kHz 每拍只来一两滴，
                                                     * 没有它窗龄凑不齐、估计器抱死旧值 */
static SpeedEst_Config_t s_se_cfg;              /* 装参留底：掉队复位时按原参重装 */
static Encoder_Sample_t s_enc_buf[ENC_RING_DEPTH];  /* TakeNew 落点（Bsp 类型） */
static SpeedEst_Sample_t s_se_buf[ENC_RING_DEPTH];  /* 分层搬运（算法类型，禁裸转） */

static PIDInstance s_speed_pid[MOTOR_CH_COUNT]; /* 每路一个实例（W2.1 PID） */
static float s_ref_rpm[MOTOR_CH_COUNT];         /* 设定转速（duty 语义符号） */
static float s_speed_rpm[MOTOR_CH_COUNT];       /* 最近一拍折算的实测转速 */
static uint32_t s_evt_last_us;                  /* 黑匣子限频时间戳 */
static uint32_t s_mt_drops[MOTOR_CH_COUNT];     /* 采样环掉队累计（板验 bisect 观测量） */
static uint32_t s_dbg_stale_seen[MOTOR_CH_COUNT];   /* 病灶上报的已见 stale 计数 */
static bool s_speed_inited;                     /* Motor_Init 已装参（防裸调闭环） */

static void SpeedLoopInit(void)
{
    PID_Init_Config_s c;
    c.Kp = ROBOT_MOTOR_SPEED_KP;
    c.Ki = ROBOT_MOTOR_SPEED_KI;
    c.Kd = ROBOT_MOTOR_SPEED_KD;
    c.Kff = ROBOT_MOTOR_SPEED_KFF;
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
    }
    s_se_cfg.ppr = (float)ROBOT_ENC_PPR;
    s_se_cfg.win_min_us = ROBOT_MT_WIN_MIN_US;
    s_se_cfg.stale_us = ROBOT_MT_STALE_US;
    s_se_cfg.lpf_rc_us = ROBOT_MT_LPF_RC_US;
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        SpeedEst_Init(&s_speed_est[i], &s_se_cfg);
        SpeedEstDrip_Init(&s_se_drip[i]);
    }
    s_evt_last_us = 0u;
    s_speed_inited = true;
}

/* IN=00（STOP 滑行）：duty 0 与 Disable 共用的关断写法 */
static void CoastChannel(const MotorMap_t *m)
{
    Gpio_Reset(s_dir_a[m->dir_idx]);
    Gpio_Reset(s_dir_b[m->dir_idx]);
    Pwm_SetDuty(m->pwm_ch, 0.0f);
}

/* 两块 TB6612 的 STBY 同开同关：只拉起一片，另一片上的电机给再大占空比
 * 也无力矩；只关一片则关不断电。开/关都两片一起动，不留半开关状态 */
static void StbyAll(bool on)
{
    (on ? Gpio_Set : Gpio_Reset)(PIN_PWR_STBY);
    (on ? Gpio_Set : Gpio_Reset)(PIN_PWR_STBY2);
}

bool Motor_Init(void)
{
    Pwm_InitAll();                  /* 幂等：只 HAL_TIM_PWM_Start，保持上电 compare=0 */
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        CoastChannel(&s_map[i]);
    }
    StbyAll(false);                 /* 保持关断（CubeMX 复位已低，此处幂等） */
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

float Motor_GetIout(MotorCh_t ch)
{
    return (ch < MOTOR_CH_COUNT) ? s_speed_pid[(uint8_t)ch].Iout : 0.0f;
}

uint32_t Motor_MtDrops(MotorCh_t ch)
{
    return (ch < MOTOR_CH_COUNT) ? s_mt_drops[(uint8_t)ch] : 0u;
}

void Motor_SetTune(float kp, float ki, float kd, float kff)
{
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        s_speed_pid[i].Kp = kp;
        s_speed_pid[i].Ki = ki;
        s_speed_pid[i].Kd = kd;
        s_speed_pid[i].Kff = kff;
    }
}

void Motor_GetTune(float *kp, float *ki, float *kd, float *kff)
{
    *kp = s_speed_pid[0].Kp;
    *ki = s_speed_pid[0].Ki;
    *kd = s_speed_pid[0].Kd;
    *kff = s_speed_pid[0].Kff;
}

void Motor_Enable(void)
{
    StbyAll(true);
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
    StbyAll(false);                 /* 后写 STBY：先清 duty 后断电，次序即保险 */
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

    /* ① 测速（M/T）：每拍取走采样环新样本 → 边沿吸附窗折算 rpm，1kHz 全程
     *    新鲜（旧 M 法 20ms 才刷新一次、低速一格台阶 1.2rpm）。GetSpeedRpm
     *    只读缓存；Encoder_Read 直读仍在但闭环不消费（旁路观察）。
     *    符号：×ROBOT_MOTOR_SIGN 折算到 duty 语义——闭环后正 ref → 正 duty →
     *    正实测，与开环符号交叉判读"O=一致"同口径，保证负反馈 */
    uint32_t now_us = Bsp_GetUs();
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        bool dropped = false;
        uint8_t n = Encoder_TakeNew(s_enc_ch[i], s_enc_buf,
                                    (uint8_t)ENC_RING_DEPTH, &dropped);
        if (dropped)
        {
            /* 掉队 = 样本流断档（64 格 + 1kHz 消费下不该发生）：按原参重装
             * 估计器和滴喂历史，宁可几拍读 0，也不拿跨断档的窗编假转速 */
            s_mt_drops[i]++;
            SpeedEst_Init(&s_speed_est[i], &s_se_cfg);
            SpeedEstDrip_Init(&s_se_drip[i]);
        }
        for (uint8_t j = 0u; j < n; j++)
        {
            s_se_buf[j].cnt = s_enc_buf[j].cnt;
            s_se_buf[j].us = s_enc_buf[j].us;
        }
        float rpm = SpeedEstDrip_Update(&s_se_drip[i], &s_speed_est[i],
                                        s_se_buf, n, now_us) *
                    (float)ROBOT_MOTOR_SIGN;

        /* 病灶上报：stale 在"产出健康"时触发就是病，把 offending 数字
         * 打出来（300ms 限频与 EVT 共用节拍；age 打 µs 原值——0xFFFFFFFF
         * 这类回绕值除以 1000 会伪装成普通毫秒数） */
        if (s_speed_est[i].dbg_stale_cnt != s_dbg_stale_seen[i] &&
            (now_us - s_evt_last_us) > 300000u)
        {
            s_dbg_stale_seen[i] = s_speed_est[i].dbg_stale_cnt;
            Log_Printf("[STALE] CH%u age_us=%lu n=%u cnt=%lu\r\n",
                       (unsigned)(i + 1u),
                       (unsigned long)s_speed_est[i].dbg_stale_age_us,
                       (unsigned)s_speed_est[i].dbg_stale_n,
                       (unsigned long)s_speed_est[i].dbg_stale_cnt);
            s_evt_last_us = now_us;
        }

        /* 事件黑匣子：宽触发（读数较上一拍掉超三成+10rpm，或涨超三成
         * +10rpm）——M/T 后"上一拍"只隔 1ms，够得着这条线的只有真毛刺/
         * 真故障的陡沿；缓变深谷不在此列（要抓缓变看 STAT/VOFA 曲线），
         * 300ms 限频不变 */
        float prev = s_speed_rpm[i];
        bool dip = (rpm < prev * 0.7f - 10.0f);
        bool jump = (rpm > prev * 1.3f + 10.0f);
        if (prev > 20.0f && (dip || jump) &&
            (now_us - s_evt_last_us) > 300000u)
        {
            long d10 = (long)(s_duty_dbg[i] * 100.0f);
            Log_Printf("[EVT] CH%u rpm=%ld<- %ld "
                       "duty=%ld%% Iout=%ld/1000 ref=%ld\r\n",
                       (unsigned)(i + 1u),
                       (long)rpm, (long)prev, d10,
                       (long)(s_speed_pid[i].Iout * 1000.0f),
                       (long)s_ref_rpm[i]);
            s_evt_last_us = now_us;
        }
        s_speed_rpm[i] = rpm;
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
            /* 符号与绝对值拆开印：负数直接 %ld.%ld 会把负分数带出来
             * （"ref=-52.-4"，应为 "-52.4"） */
            if (ref10 < 0)
            {
                ref10 = -ref10;
            }
            if (spd10 < 0)
            {
                spd10 = -spd10;
            }
            Log_Printf("[MOTOR] CH%u 堵转闩置起 ref=%c%ld.%ld rpm 实测=%c%ld.%ld rpm\r\n",
                       (unsigned)(i + 1u),
                       (s_ref_rpm[i] < 0.0f) ? '-' : '+', ref10 / 10, ref10 % 10,
                       (s_speed_rpm[i] < 0.0f) ? '-' : '+', spd10 / 10, spd10 % 10);
            s_speed_pid[i].ERRORHandler.ERRORType = PID_ERROR_NONE;
            s_speed_pid[i].ERRORHandler.ERRORCount = 0u;
        }

        Motor_SetDuty((MotorCh_t)i, out);
    }
}
