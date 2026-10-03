/**
 * @file    test_motor.c
 * @brief   S5 测试项 6=MOTOR：四路开环正反转阶梯 + 编码器符号交叉验证
 *
 * 验收标准（Modules 层开发规划 §3.1）：四路开环正反转（duty 阶梯
 * 0.2/0.5/1.0），方向与编码器计数符号交叉验证；Disable 后手转轮无阻力矩。
 *
 * 序列（自动循环，2s/步，每步串口打印判据）：
 *   Enable → 正转阶梯 +20%/+50%/+100% → 反转阶梯 -20%/-50%/-100% →
 *   Disable（STBY 断）→ 提示手转车轮（应无阻力矩，编码器计数随手动变化）→
 *   循环。
 *
 * 符号交叉判读（O=一致 / X=相反需对调）：每档打印四路编码器增量，与 duty
 * 符号逐通道比对——X 的通道 = 该路电机线接反或映射表不对，处置：对调该路
 * 电机两线，或改 robot_config.h 的 ROBOT_MOTOR_CHx_DIR/PWM（映射一处改，
 * 代码零改动）。基准前提：占位映射 = 按序号直连，编码器 CHx 与电机 CHx
 * 的对应关系同样以实机布线为准核对。
 */
#include "test_bench.h"
#include "motor.h"
#include "bsp_encoder.h"
#include "bsp_sys.h"
#include "bsp_log.h"

/* 增量符号与 duty 符号交叉判定：'O'=一致 'X'=相反 ' '=无判据 */
static char SignVerdict(int32_t inc, float duty)
{
    if (duty == 0.0f || inc == 0)
    {
        return ' ';
    }
    return ((duty > 0.0f) == (inc > 0)) ? 'O' : 'X';
}

void Test_Motor_Init(void)
{
    Encoder_InitAll();
    Motor_Init();       /* STBY 保持低：不出力，安全 */
    Log_Printf("[T-MOTOR] 6=MOTOR：Enable→正/反转阶梯(20/50/100%%)→Disable手转，"
               "2s/步；符号判读 O=一致 X=相反需对调\r\n");
}

void Test_Motor_Poll(void)
{
    static uint32_t s_last;
    static uint8_t  s_step;         /* 0..5 = ±阶梯，6 = Disable 手转 */
    static float    s_prev_duty;    /* 刚结束窗口内施加的 duty（首窗=0 待机） */
    uint32_t now = Bsp_GetMs();
    if ((now - s_last) < 2000u)
    {
        return;
    }
    s_last = now;

    static const float s_ladder[] = { 0.2f, 0.5f, 1.0f, -0.2f, -0.5f, -1.0f };
    const uint8_t n = (uint8_t)(sizeof(s_ladder) / sizeof(s_ladder[0]));

    /* 先归账：上一窗口四路编码器增量 × duty 符号交叉判定 */
    int32_t inc[MOTOR_CH_COUNT];
    for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
    {
        inc[i] = Encoder_Read(ENC_CH1 + i);
    }
    Log_Printf("[T-MOTOR] 增量 CH1=%+ld%c CH2=%+ld%c CH3=%+ld%c CH4=%+ld%c\r\n",
               (long)inc[0], SignVerdict(inc[0], s_prev_duty),
               (long)inc[1], SignVerdict(inc[1], s_prev_duty),
               (long)inc[2], SignVerdict(inc[2], s_prev_duty),
               (long)inc[3], SignVerdict(inc[3], s_prev_duty));

    /* 再进下一步 */
    s_step = (uint8_t)((s_step + 1u) % (n + 1u));
    if (s_step < n)
    {
        float duty = s_ladder[s_step];
        Motor_Enable();
        for (uint8_t i = 0u; i < MOTOR_CH_COUNT; i++)
        {
            Motor_SetDuty((MotorCh_t)i, duty);
        }
        s_prev_duty = duty;
        Log_Printf("[T-MOTOR] step=%u duty=%+.0f%% 四路同步\r\n",
                   s_step, duty * 100.0f);
    }
    else
    {
        Motor_Disable();
        s_prev_duty = 0.0f;
        Log_Printf("[T-MOTOR] 已 Disable（STBY 断）：请手转车轮——应无阻力矩，"
                   "下窗增量即手动计数\r\n");
    }
}
