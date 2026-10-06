/**
 * @file    pid.c
 * @brief   PID 控制器实现（移植自 control-2026 Modules/algorithm/controller.c）
 *
 * 改动清单见 pid.h 头部；各优化环节函数与原版一一对应，
 * 计算顺序（堵转检测 → 算间隔 → P/I/D → 滤波 → 限幅）原样保留。
 */
#include "pid.h"
#include "bsp_sys.h"
#include <math.h>

/* 两次计算落在同一微秒时的最小间隔（秒）：正常控制环 1kHz、间隔 1ms=1000µs，
 * 这个兜底值对它没有影响，只在病态调用时防止微分项除以 0 */
#define PID_DT_MIN_S    1e-6f

/* ---------------------------- PID 优化环节实现 ---------------------------- */

/* 梯形积分：这次和上次的误差取平均再积，比只算这次的矩形积分平缓些 */
static void f_Trapezoid_Intergral(PIDInstance *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2.0f) * pid->dt;
}

/* 变速积分：误差越大积分越保守（怕大误差时积分冲太猛）；
 * 只在积分正朝着误差同方向越积越多时起作用 */
static void f_Changing_Integration_Rate(PIDInstance *pid)
{
    if (pid->Err * pid->Iout > 0.0f)
    {
        if (fabsf(pid->Err) <= pid->CoefB)
            return; /* 误差不大：全速积 */
        if (fabsf(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - fabsf(pid->Err) + pid->CoefB) / pid->CoefA; /* 误差偏大：按比例减速 */
        else
            pid->ITerm = 0.0f; /* 误差太大：这次不积 */
    }
}

static void f_Integral_Limit(PIDInstance *pid)
{
    /* 这两个临时量必须是函数内的局部变量——原版用了 static（所有实例
     * 共用一份），几个 PID 一起算会互相覆盖 */
    float temp_Iout = pid->Iout + pid->ITerm;
    float temp_Output = pid->Pout + pid->Iout + pid->Dout;

    if (fabsf(temp_Output) > pid->MaxOut)   /* 输出已经顶到限幅 */
    {
        if (pid->Err * pid->Iout > 0.0f)    /* 而积分还在朝误差方向继续涨 */
        {
            pid->ITerm = 0.0f;              /* 这次的新增积分作废，防止越顶越积 */
        }
    }

    if (temp_Iout > pid->IntegralLimit)     /* 累计积分顶到上限 */
    {
        pid->ITerm = 0.0f;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0.0f;
        pid->Iout = -pid->IntegralLimit;
    }
}

/* 微分先行：只对"测量值的变化"做微分——指令突变（比如摇杆猛推）不会
 * 在输出上打出一个尖峰 */
static void f_Derivative_On_Measurement(PIDInstance *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

/* 微分滤波：微分容易放大采样噪声，用一阶低通抹平 */
static void f_Derivative_Filter(PIDInstance *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

/* 输出滤波：输出也过一阶低通 */
static void f_Output_Filter(PIDInstance *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

/* 输出限幅：输出不许超过 MaxOut */
static void f_Output_Limit(PIDInstance *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

/* 堵转检测：指令很大、电机却长时间没动（误差一直剩 95% 以上）→ 计数；
 * 电机追上了 → 计数归零；连续超过 500 次（1kHz 下即 0.5 秒）→ 判定堵转。
 * 判定后只把标志置起，之后怎么处理、什么时候清除是使用方的事 */
static void f_PID_ErrorHandle(PIDInstance *pid)
{
    /* 两种情况"动没动"没有意义，直接跳过：输出接近 0（根本没给出力，
     * 电机当然不动）、指令接近 0（本来就让它停）。跳过时计数既不加也
     * 不清——堵转嫌疑不因为"歇了一会儿"被洗白（保留原版行为） */
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
        return;

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f) /* 误差还剩 95% 以上 = 基本没动 */
    {
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;   /* 动起来了，历史清零 */
    }

    if (pid->ERRORHandler.ERRORCount > 500)
    {
        pid->ERRORHandler.ERRORType = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ----------------------------- 外部算法接口 ------------------------------ */

void PIDInit(PIDInstance *pid, const PID_Init_Config_s *config)
{
    /* 按名字逐字段赋值（原版用 memcpy 把配置字节整体拍进实例，要求两个
     * 结构体的内存排列一模一样，改错字段也不报错——这里一个个来，
     * 编译器帮着查） */
    pid->Kp = config->Kp;
    pid->Ki = config->Ki;
    pid->Kd = config->Kd;
    pid->MaxOut = config->MaxOut;
    pid->DeadBand = config->DeadBand;
    pid->Improve = config->Improve;
    pid->IntegralLimit = config->IntegralLimit;
    pid->CoefA = config->CoefA;
    pid->CoefB = config->CoefB;
    pid->Output_LPF_RC = config->Output_LPF_RC;
    pid->Derivative_LPF_RC = config->Derivative_LPF_RC;

    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->dt = 0.0f;
    pid->ERRORHandler.ERRORCount = 0u;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    /* 记下初始化时刻：第一次计算也能得到"距初始化过了多久"当间隔 */
    pid->last_us = Bsp_GetUs();
}

float PIDCalculate(PIDInstance *pid, float measure, float ref)
{
    /* 堵转检测（看的是上一次计算的结果，原版顺序保留） */
    if (pid->Improve & PID_ErrorHandle)
        f_PID_ErrorHandle(pid);

    /* 算这次距上次的间隔：微秒数相减（计数器转满归零后相减依然正确）；
     * 同一微秒里调了两次时按最小间隔算，防止微分项除以 0 */
    uint32_t now_us = Bsp_GetUs();
    float dt = (float)(now_us - pid->last_us) * 1e-6f;
    pid->last_us = now_us;
    if (dt < PID_DT_MIN_S)
    {
        dt = PID_DT_MIN_S;
    }
    pid->dt = dt;

    /* 记下这次的测量值和指令，算出误差 */
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (fabsf(pid->Err) > pid->DeadBand) /* 误差在死区外才计算 */
    {
        /* 基本位置式 PID：P 看当前误差，I 累积历史误差，D 看误差变化快慢 */
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         /* 积分累计 */
        pid->Output = pid->Pout + pid->Iout + pid->Dout; /* 三项加总 */

        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        f_Output_Limit(pid);
    }
    else /* 误差在死区内：输出清零、这次新增积分清零（历史累计的积分保留） */
    {
        pid->Output = 0.0f;
        pid->ITerm = 0.0f;
    }

    /* 把这次的数据存起来给下次用 */
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}
