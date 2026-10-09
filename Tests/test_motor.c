/**
 * @file    test_motor.c
 * @brief   S5 测试项 6=MOTOR：四路开环正反转阶梯 + 编码器符号交叉验证
 *          + W2.3 速度环阶跃联验
 *
 * 验收标准（Modules 层开发规划 §3.1 / W2.3 排期）：
 *  - S5 开环：四路正反转（duty 阶梯 0.2/0.5/1.0），方向与编码器计数符号
 *    交叉验证；Disable 后手转轮无阻力矩；
 *  - W2.3 闭环：速度阶跃响应无超调振荡（整定参数记录进 robot_config.h）。
 *
 * 序列（自动循环，每步 2s，两阶段共用同一 2s 门控时间戳）：
 *   开环阶梯 ±20%/±50%/±100%（打印符号判读 O/X）→ Disable 手转（应无阻力矩）
 *   → 速度阶跃 +30/+60/−30/0 rpm（ROBOT_MOTOR_SPEED_TEST_LIST）→ 回开环循环。
 *
 * 【单电机测试态 2026-10-07】手头只有一路电机/编码器，符号判读与速度阶跃
 * 只走 CH4（其余三路 ref 保持 0：悬空编码器的乱数不进闭环判据，空通道
 * 出随机 duty 也无电机可驱动）。恢复四路联调：判读打印与 ref 下发处把
 * TEST_MOTOR_CH 换回全通道即可。
 *
 * ⚠ 闭环前置条件：开环阶段 CH4 符号判读必须 O——编码器符号 × 电机方向
 *   不一致 = 正反馈自激（规划明令风险）。先对调该路电机两线或改
 *   robot_config.h 的 ROBOT_MOTOR_CH4_ENC/PWM/DIR 映射，再进速度阶段。
 *   编码器现象诡异先怀疑共地（BSP 坑#1 前科），别先怀疑软件。
 *
 * 速度阶段实现：测试台轮询 10ms 一拍，带不动 1kHz——每拍内跑 10 次
 * Motor_SpeedLoopUpdate + osDelay(1) 突发，闭环节拍与业务态 ChassisTask
 * 一致。每 200ms 打印实测转速（0.1rpm 定点，newlib-nano 无 %f），读数走
 * Motor_GetSpeedRpm（编码器唯一读者是速度环，直接 Encoder_Read 会抢走
 * 测速增量）。
 */
#include "test_bench.h"
#include "cmsis_os.h"
#include "motor.h"
#include "bsp_encoder.h"
#include "bsp_sys.h"
#include "bsp_log.h"
#include "bsp_vofa.h"

/* 增量符号与 duty 符号交叉判定：'O'=一致 'X'=相反 ' '=无判据 */
static char SignVerdict(int32_t inc, float duty)
{
    if (duty == 0.0f || inc == 0)
    {
        return ' ';
    }
    return ((duty > 0.0f) == (inc > 0)) ? 'O' : 'X';
}

/* rpm 以 0.1rpm 定点打印（newlib-nano 默认无 %f）：+12.3 → "+12.3" */
static void Log_RpmX10(const char *tag, float rpm)
{
    long x10 = (long)(rpm * 10.0f);
    bool neg = (x10 < 0);
    unsigned long a = (unsigned long)(neg ? -x10 : x10);
    Log_Printf("%s%c%lu.%lu", tag, neg ? '-' : '+', a / 10u, a % 10u);
}

/* 开环阶梯（duty）与速度阶跃（rpm）序列；步序：
 * 0..5 开环 → 6 Disable 手转 → 7.. 速度阶跃 → 回 0 循环 */
static const float s_ladder[] = { 0.2f, 0.5f, 1.0f, -0.2f, -0.5f, -1.0f };
static const float s_speed_steps[] = ROBOT_MOTOR_SPEED_TEST_LIST;
#define LADDER_N     (sizeof(s_ladder) / sizeof(s_ladder[0]))
#define SPEED_N      (sizeof(s_speed_steps) / sizeof(s_speed_steps[0]))
#define SPEED_FIRST  ((uint8_t)(LADDER_N + 1u))
#define STEP_TOTAL   ((uint8_t)(SPEED_FIRST + SPEED_N))

/* 单电机测试态：符号判读/速度阶跃只走这一路（恢复四路改回全通道循环） */
#define TEST_MOTOR_CH    MOTOR_CH4
#define TEST_MOTOR_IDX   3u

/* 步进机状态：步号 + 两阶段共用的 2s 门控 / 200ms 打印节拍。
 * 共用门控的原因：两阶段各有 static 时间戳的话，阶段切换时对方 8s 没更新，
 * 回来第一拍就过门，吞掉一个步进窗（COM16 实测踩过） */
static uint8_t  g_step;
static uint32_t s_gate_last;
static uint32_t s_print_last;
static float    s_prev_duty;    /* 刚结束窗口内施加的 duty（首窗=0 待机） */

/* 施加开环阶梯第 idx 步（四路同步），供开环推进与速度阶段回卷共用 */
static void ApplyLadderStep(uint8_t idx)
{
    float duty = s_ladder[idx];
    Motor_Enable();
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        Motor_SetDuty((MotorCh_t)i, duty);
    }
    s_prev_duty = duty;
    Log_Printf("[T-MOTOR] step=%u duty=%+ld%% 四路同步\r\n",
               idx, (long)(duty * 100.0f));
}

/* 速度阶段每拍：1kHz 突发 + 200ms 读数打印 + 2s 进下一阶跃 */
static void SpeedPhaseTick(void)
{
    uint8_t cur = (uint8_t)(g_step - SPEED_FIRST);  /* 当前档（0..SPEED_N-1） */

    for (uint8_t i = 0u; i < 10u; i++)      /* 补齐 1kHz：10ms 轮询 × 10 次 */
    {
        Motor_SpeedLoopUpdate();
        /* VOFA+ JustFloat：CH4 {ref, 实测, duty, Iout} 四通道 1kHz 流，
         * 忙丢帧不阻塞控制环；通道顺序即 VOFA+ 通道 0..3 */
        float vofa[4] = {
            s_speed_steps[cur],
            Motor_GetSpeedRpm(TEST_MOTOR_CH),
            Motor_GetDuty(TEST_MOTOR_CH),
            Motor_GetIout(TEST_MOTOR_CH),
        };
        (void)Bsp_Vofa_SendFloats(vofa, 4u);
        osDelay(1u);
    }

    uint32_t now = Bsp_GetMs();
    if ((now - s_print_last) >= 200u)
    {
        s_print_last = now;
        Log_Printf("[T-MOTOR] 实测(0.1rpm)");
        Log_RpmX10(" CH", Motor_GetSpeedRpm(TEST_MOTOR_CH));
        Log_Printf(" duty=%+ld%%\r\n", (long)(Motor_GetDuty(TEST_MOTOR_CH) * 100.0f));
    }

    if ((now - s_gate_last) < 2000u)
    {
        return;
    }
    s_gate_last = now;

    uint8_t idx = (uint8_t)(g_step - SPEED_FIRST);
    uint8_t next = (uint8_t)((idx + 1u) % SPEED_N);
    if (next == 0u)                             /* 速度阶段末：断电回开环循环 */
    {
        Motor_Disable();                        /* 同时清 ref/积分态 */
        g_step = 0u;                            /* 步号回开环起点（否则永远圈在速度阶段） */
        ApplyLadderStep(0u);                    /* 回卷即施加 step0，否则 step0 窗口空跑 */
        Log_Printf("[T-MOTOR] 速度阶段结束，回开环循环\r\n");
    }
    else
    {
        g_step = (uint8_t)(SPEED_FIRST + next); /* 步号同步推进（不推则 idx 恒 0 卡死） */
        Motor_SetSpeedRpm(TEST_MOTOR_CH, s_speed_steps[next]);
        Log_Printf("[T-MOTOR] 速度阶跃");
        Log_RpmX10(" CH", s_speed_steps[next]);
        Log_Printf(" rpm\r\n");
    }
}

void Test_Motor_Init(void)
{
    Encoder_InitAll();
    Motor_Init();       /* STBY 保持低：不出力，安全（速度环同时装参清态） */
    Bsp_Vofa_Init();    /* UART4 提速 921600，VOFA+ JustFloat 整定流专用 */
    /* 打印固件编译进的速度环参数（排查"改了参数没重编/没烧上"） */
    Log_Printf("[T-MOTOR] 速度环参数 Kp=%ld/10000 Ki=%ld/10000 CALC=%ums "
               "PPR=%ld\r\n",
               (long)(ROBOT_MOTOR_SPEED_KP * 10000.0f),
               (long)(ROBOT_MOTOR_SPEED_KI * 10000.0f),
               (unsigned)ROBOT_MOTOR_SPEED_CALC_MS,
               (long)ROBOT_ENC_PPR);
    /* Bsp_GetUs 自检：PID 的 dt 靠它（DWT 需 Bsp_Init 里 LAR 解锁，坑#15） */
    {
        uint32_t us0 = Bsp_GetUs();
        osDelay(2u);
        uint32_t us1 = Bsp_GetUs();
        Log_Printf("[T-MOTOR] GetUs 自检 Δ=%lu us（期望 ≈2000）\r\n",
                   (unsigned long)(us1 - us0));
    }
    Log_Printf("[T-MOTOR] 6=MOTOR：开环阶梯(20/50/100%%)→Disable手转→速度阶跃"
               "(+30/+60/-30/0rpm)，2s/步；单电机测试态=CH4，符号判读 O=一致"
               " X=相反需对调，O 才允许闭环\r\n");
}

void Test_Motor_Poll(void)
{
    uint32_t now = Bsp_GetMs();

    if (g_step >= SPEED_FIRST)
    {
        SpeedPhaseTick();
        return;
    }

    if ((now - s_gate_last) < 2000u)
    {
        return;
    }
    s_gate_last = now;

    const uint8_t n = (uint8_t)LADDER_N;

    /* 先归账：上一窗口编码器增量；单电机测试态只判 CH4（其余悬空乱跳无判据） */
    int32_t inc[MOTOR_CH_COUNT];
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        inc[i] = Encoder_Read(ENC_CH1 + i);
    }
    Log_Printf("[T-MOTOR] 增量 CH1=%+ld CH2=%+ld CH3=%+ld CH4=%+ld "
               "符号判读(CH4)=%c\r\n",
               (long)inc[0], (long)inc[1], (long)inc[2], (long)inc[3],
               SignVerdict(inc[TEST_MOTOR_IDX], s_prev_duty));

    /* 再进下一步 */
    g_step = (uint8_t)((g_step + 1u) % STEP_TOTAL);
    if (g_step < n)
    {
        ApplyLadderStep(g_step);
    }
    else if (g_step == n)
    {
        Motor_Disable();
        s_prev_duty = 0.0f;
        Log_Printf("[T-MOTOR] 已 Disable（STBY 断）：请手转车轮——应无阻力矩，"
                   "下窗增量即手动计数\r\n");
    }
    else    /* 进入速度阶段：闭环前最后一道闸——重装 PID（清积分/记时起点） */
    {
        Motor_Init();       /* 幂等重装：手转阶段残留的计数快照/积分态全清 */
        Motor_Enable();
        Motor_SetSpeedRpm(TEST_MOTOR_CH, s_speed_steps[0]);   /* 单电机：只 CH4 */
        s_print_last = now;
        Log_Printf("[T-MOTOR] 进速度阶段（闭环！前提：CH4 符号判读 O）"
                   "阶跃 %+ld rpm\r\n", (long)s_speed_steps[0]);
    }
}
