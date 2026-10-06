/**
 * @file    test_pid.c
 * @brief   PID 单测（W2.1，host 编译）——规格固化的边界用例：
 *          P/I/D 基本量纲、MaxOut/IntegralLimit 双重限幅、死区、梯形积分、
 *          微分先行/微分滤波、变速积分、堵转计数与闩锁、dt=0、err 跳变、
 *          多实例隔离（static 污染回归）。
 */
#include "bsp_sys.h"
#include "bsp_sys_stub.h"
#include "pid.h"
#include "test_util.h"
#include <math.h>

/* 1kHz 节拍：时钟前进一拍再计算（模拟速度环任务周期） */
static float Step1k(PIDInstance *pid, float measure, float ref)
{
    Stub_Time_AdvanceUs(1000u);
    return PIDCalculate(pid, measure, ref);
}

/* P 基本量纲 + 输出限幅 + 双向符号 */
static void TestP_GainAndClamp(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 2.0f, .Ki = 0.0f, .Kd = 0.0f,
        .MaxOut = 0.5f, .DeadBand = 0.0f,
        .Improve = PID_IMPROVE_NONE,
    };
    PIDInstance pid;
    PIDInit(&pid, &cfg);

    float out = Step1k(&pid, 0.0f, 1.0f); /* err=1 → Pout=2 → 钳 0.5 */
    CHECK_NEAR(out, 0.5, 1e-6);

    out = Step1k(&pid, 0.8f, 1.0f); /* err=0.2 → 0.4 */
    CHECK_NEAR(out, 0.4, 1e-6);

    out = Step1k(&pid, 0.0f, -1.0f); /* 负向：err=-1 → -2 → 钳 -0.5 */
    CHECK_NEAR(out, -0.5, 1e-6);
}

/* 积分累加 + IntegralLimit 双侧钳位 */
static void TestIntegral_LimitClamp(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 0.0f, .Ki = 10.0f, .Kd = 0.0f,
        .MaxOut = 1000.0f, .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = 5.0f,
    };
    PIDInstance pid;
    PIDInit(&pid, &cfg);

    for (int i = 0; i < 700; i++) /* 0.01/拍 → ~500 拍顶到 +5 */
        Step1k(&pid, 0.0f, 1.0f);
    CHECK_NEAR(pid.Iout, 5.0, 1e-5);
    CHECK_NEAR(pid.Output, 5.0, 1e-5);

    for (int i = 0; i < 1200; i++) /* 反向积分 → 顶到 -5 */
        Step1k(&pid, 0.0f, -1.0f);
    CHECK_NEAR(pid.Iout, -5.0, 1e-4);
    CHECK_NEAR(pid.Output, -5.0, 1e-4);

    /* 重初始化清态：Iout 归零（消费方复用实例的前提） */
    PIDInit(&pid, &cfg);
    CHECK_NEAR(pid.Iout, 0.0, 1e-9);
}

/* 死区：|err|≤DeadBand 输出/ITerm 清零；出死区恢复计算 */
static void TestDeadBand(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f,
        .MaxOut = 100.0f, .DeadBand = 0.1f,
        .Improve = PID_IMPROVE_NONE,
    };
    PIDInstance pid;
    PIDInit(&pid, &cfg);

    float out = Step1k(&pid, 0.95f, 1.0f); /* |err|=0.05 ≤ 0.1 */
    CHECK_NEAR(out, 0.0, 1e-9);
    CHECK_NEAR(pid.ITerm, 0.0, 1e-9);

    out = Step1k(&pid, 0.0f, 1.0f); /* |err|=1 出死区 */
    CHECK_NEAR(out, 1.0, 1e-6);
}

/* 梯形积分：ITerm = Ki*(Err+Last_Err)/2*dt */
static void TestTrapezoidIntegral(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 0.0f, .Ki = 10.0f, .Kd = 0.0f,
        .MaxOut = 1e6f, .DeadBand = 0.0f,
        .Improve = PID_Trapezoid_Intergral,
    };
    PIDInstance pid;
    PIDInit(&pid, &cfg);

    Step1k(&pid, 0.0f, 1.0f); /* (1+0)/2 → ITerm=0.005 */
    CHECK_NEAR(pid.Iout, 0.005, 1e-7);

    Step1k(&pid, 0.0f, 1.0f); /* (1+1)/2 → ITerm=0.01 → Iout=0.015 */
    CHECK_NEAR(pid.Iout, 0.015, 1e-7);
}

/* 微分先行：Dout = Kd*(Last_Measure-Measure)/dt，抵抗反馈量变化 */
static void TestDerivativeOnMeasurement(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 0.0f, .Ki = 0.0f, .Kd = 2.0f,
        .MaxOut = 1e6f, .DeadBand = 0.0f,
        .Improve = PID_Derivative_On_Measurement,
    };
    PIDInstance pid;
    PIDInit(&pid, &cfg);

    Step1k(&pid, 1.0f, 0.0f); /* measure 0→1（上拍 0） */
    CHECK_NEAR(pid.Dout, -2000.0, 1e-3);

    Step1k(&pid, 1.0f, 0.0f); /* 反馈不变 → Dout=0 */
    CHECK_NEAR(pid.Dout, 0.0, 1e-6);

    Step1k(&pid, 0.5f, 0.0f); /* measure 1→0.5 → +1000 */
    CHECK_NEAR(pid.Dout, 1000.0, 1e-3);
}

/* 微分滤波：一阶低通，Dout 向原始微分值渐进 */
static void TestDerivativeFilter(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 0.0f, .Ki = 0.0f, .Kd = 1.0f,
        .MaxOut = 1e6f, .DeadBand = 0.0f,
        .Improve = PID_Derivative_On_Measurement | PID_DerivativeFilter,
        .Derivative_LPF_RC = 0.001f, /* RC=1ms，dt=1ms → 首拍折半 */
    };
    PIDInstance pid;
    PIDInit(&pid, &cfg);

    Step1k(&pid, 1.0f, 0.0f); /* 原始 -1000 → 滤波后 -500 */
    CHECK_NEAR(pid.Dout, -500.0, 1e-3);

    Step1k(&pid, 1.0f, 0.0f); /* 原始 0 → 向 0 渐进 -250 */
    CHECK_NEAR(pid.Dout, -250.0, 1e-3);
}

/* 变速积分：|err|≤B 全速，B<|err|≤A+B 降速，>A+B 不积 */
static void TestChangingIntegrationRate(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 0.0f, .Ki = 10.0f, .Kd = 0.0f,
        .MaxOut = 1e6f, .DeadBand = 0.0f,
        .Improve = PID_ChangingIntegrationRate,
        .CoefA = 100.0f, .CoefB = 50.0f,
    };
    PIDInstance pid;
    PIDInit(&pid, &cfg);

    Step1k(&pid, 0.0f, 10.0f); /* Iout=0 时 Err*Iout 不大于 0 → 全速：0.1 */
    CHECK_NEAR(pid.Iout, 0.1, 1e-6);

    Step1k(&pid, 0.0f, 75.0f); /* 50<75≤150 → ×0.75：ITerm=0.5625 */
    CHECK_NEAR(pid.Iout, 0.6625, 1e-4);

    Step1k(&pid, 0.0f, 200.0f); /* >150 → ITerm=0，Iout 冻结 */
    CHECK_NEAR(pid.Iout, 0.6625, 1e-6);
    CHECK_NEAR(pid.ITerm, 0.0, 1e-9);
}

/* 堵转检测：>95% 残差持续计数 → 闩锁；跳检条件与恢复清零 */
static void TestErrorHandle(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 1.0f, .Ki = 0.0f, .Kd = 0.0f,
        .MaxOut = 100.0f, .DeadBand = 0.0f,
        .Improve = PID_ErrorHandle,
    };
    PIDInstance pid;
    PIDInit(&pid, &cfg);

    for (int i = 0; i < 600; i++) /* 首拍 Ref=0 跳检 → 计数 599 */
        Step1k(&pid, 0.0f, 100.0f);
    CHECK(pid.ERRORHandler.ERRORCount == 599u);
    CHECK(pid.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR);

    /* 跳检①：输出近零（|Output| < MaxOut*0.001）不计数 */
    PID_Init_Config_s weak = cfg;
    weak.Kp = 0.0001f; /* Output≈0.01 < 0.1 */
    PIDInit(&pid, &weak);
    for (int i = 0; i < 600; i++)
        Step1k(&pid, 0.0f, 100.0f);
    CHECK(pid.ERRORHandler.ERRORCount == 0u);

    /* 跳检②：|Ref| < 1e-4 不计数 */
    PIDInit(&pid, &cfg);
    for (int i = 0; i < 600; i++)
        Step1k(&pid, 0.0f, 0.00001f);
    CHECK(pid.ERRORHandler.ERRORCount == 0u);

    /* 恢复：残差回落到 ≤95% 且输出非零 → 计数清零（闩锁保留，复位归消费方）。
     * 注意 ErrorHandle 用上一拍数据：回落当拍计的仍是上拍（>95%），
     * 下一拍看到新 Output/Measure 才清零 */
    PIDInit(&pid, &cfg);
    for (int i = 0; i < 600; i++)
        Step1k(&pid, 0.0f, 100.0f);
    Step1k(&pid, 10.0f, 100.0f); /* 本拍 Output=90；ErrorHandle 还在数上拍 */
    Step1k(&pid, 10.0f, 100.0f); /* 本拍起 ratio=0.9 → 计数清零 */
    CHECK(pid.ERRORHandler.ERRORCount == 0u);
    CHECK(pid.ERRORHandler.ERRORType == PID_MOTOR_BLOCKED_ERROR);
}

/* dt=0 边界：同微秒重复调用钳到 1µs，输出有限且积分按 1µs 计 */
static void TestZeroDt(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 1.0f, .Ki = 100.0f, .Kd = 0.0f,
        .MaxOut = 1e6f, .DeadBand = 0.0f,
        .Improve = PID_IMPROVE_NONE,
    };
    PIDInstance pid;
    Stub_Time_SetUs(5000u);
    PIDInit(&pid, &cfg);

    float out = PIDCalculate(&pid, 0.0f, 1.0f); /* 不推进时钟 */
    CHECK(isfinite(out));
    CHECK_NEAR(pid.dt, 1e-6, 1e-12);
    CHECK_NEAR(pid.Iout, 1e-4, 1e-9); /* Ki*err*1µs = 1e-4 */

    out = PIDCalculate(&pid, 0.0f, 1.0f); /* 再来一拍仍同微秒 */
    CHECK(isfinite(out));

    out = Step1k(&pid, 0.0f, 1.0f); /* 恢复正常节拍 dt=1ms */
    CHECK_NEAR(pid.dt, 0.001, 1e-9);
    CHECK(isfinite(out));
}

/* err 跳变边界：Pout/Dout 巨值仍有限且被 MaxOut 收口 */
static void TestErrStepJump(void)
{
    PID_Init_Config_s cfg = {
        .Kp = 10.0f, .Ki = 0.0f, .Kd = 100.0f,
        .MaxOut = 50.0f, .DeadBand = 0.0f,
        .Improve = PID_IMPROVE_NONE,
    };
    PIDInstance pid;
    PIDInit(&pid, &cfg);

    float out = Step1k(&pid, -1000.0f, 1000.0f); /* err=2000，Dout=2e8 */
    CHECK(isfinite(pid.Dout));
    CHECK_NEAR(out, 50.0, 1e-6);

    out = Step1k(&pid, 1000.0f, -1000.0f); /* 反向跳变 */
    CHECK(isfinite(out));
    CHECK_NEAR(out, -50.0, 1e-6);
}

/* 多实例隔离：不同 IntegralLimit 的两个实例交错计算互不污染 */
static void TestMultiInstanceIsolation(void)
{
    PID_Init_Config_s cfg_a = {
        .Kp = 0.0f, .Ki = 10.0f, .Kd = 0.0f,
        .MaxOut = 1e6f, .DeadBand = 0.0f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = 1.0f,
    };
    PID_Init_Config_s cfg_b = cfg_a;
    cfg_b.IntegralLimit = 5.0f;
    PIDInstance a, b;
    PIDInit(&a, &cfg_a);
    PIDInit(&b, &cfg_b);

    for (int i = 0; i < 700; i++)
    {
        Step1k(&a, 0.0f, 1.0f);
        Step1k(&b, 0.0f, 1.0f);
    }
    CHECK_NEAR(a.Iout, 1.0, 1e-5);
    CHECK_NEAR(b.Iout, 5.0, 1e-5);
}

void TestPid_Run(void)
{
    printf("[pid]\n");
    TestP_GainAndClamp();
    TestIntegral_LimitClamp();
    TestDeadBand();
    TestTrapezoidIntegral();
    TestDerivativeOnMeasurement();
    TestDerivativeFilter();
    TestChangingIntegrationRate();
    TestErrorHandle();
    TestZeroDt();
    TestErrStepJump();
    TestMultiInstanceIsolation();
}
