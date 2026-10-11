/**
 * @file    chassis_ctrl.c
 * @brief   底盘业务纯函数实现（设计说明见 chassis_ctrl.h）
 */
#include "chassis_ctrl.h"
#include <stddef.h>

/* ------------------------------ 内部小工具 ------------------------------ */

/* 保留符号压幅：把 |x| 钳到上限并带回原符号（上限为正数） */
static float ClampSym_(float x, float limit)
{
    if (x > limit)
    {
        return limit;
    }
    if (x < -limit)
    {
        return -limit;
    }
    return x;
}

/* ------------------------------ 外部接口 ------------------------------ */

void Chassis_Shape(const Chassis_Vec_t *in, Chassis_Vec_t *out)
{
    if (in == NULL || out == NULL)
    {
        return;
    }

    const float d = ROBOT_APP_CHASSIS_DEADBAND;
    const float axes[3] = { in->vx, in->vy, in->omega };
    float shaped[3];

    for (int i = 0; i < 3; i++)
    {
        float x = ClampSym_(axes[i], 1.0f);
        float mag = (x < 0.0f) ? -x : x;

        if (mag <= d)
        {
            shaped[i] = 0.0f;               /* 死区内：回中漂移全吃掉 */
        }
        else
        {
            /* 死区外重铺满量程（x=1 仍映射 1），再平方：轻推细、推满猛 */
            float t = (mag - d) / (1.0f - d);
            shaped[i] = (x < 0.0f) ? -(t * t) : (t * t);
        }
    }

    out->vx = shaped[0];
    out->vy = shaped[1];
    out->omega = shaped[2];
}

void Chassis_ToPhysical(const Chassis_Vec_t *norm, Chassis_Vec_t *phys)
{
    if (norm == NULL || phys == NULL)
    {
        return;
    }
    phys->vx = norm->vx * ROBOT_APP_VX_MAX_MPS;
    phys->vy = norm->vy * ROBOT_APP_VY_MAX_MPS;
    phys->omega = norm->omega * ROBOT_APP_W_MAX_RADPS;
}

void Chassis_MecanumIK(const Chassis_Vec_t *phys, float rpm_out[CHASSIS_WHEEL_COUNT])
{
    if (phys == NULL || rpm_out == NULL)
    {
        return;
    }

    /* 逆解（PROTOCOL.md §3）：轮角速度 rad/s = (±vx ±vy ∓L·ω)/R，
     * γ 符号 FL-1/FR+1/RL+1/RR-1 已含在式内；rad/s → rpm（×60/2π），
     * 正 = 轮子向前滚 */
    float k = 9.5492965855f / ROBOT_APP_MECANUM_R_M;
    rpm_out[0] = k * (phys->vx - phys->vy - ROBOT_APP_MECANUM_L_M * phys->omega);
    rpm_out[1] = k * (phys->vx + phys->vy + ROBOT_APP_MECANUM_L_M * phys->omega);
    rpm_out[2] = k * (phys->vx + phys->vy - ROBOT_APP_MECANUM_L_M * phys->omega);
    rpm_out[3] = k * (phys->vx - phys->vy + ROBOT_APP_MECANUM_L_M * phys->omega);

    /* 饱和处理：任一轮超速则四轮等比缩，方向不变（等效"整车降速"而非削单轮）。
     * ⚠ 与 Modules/motor 的单通道钳位不是重复：本缩放保麦轮四轮比例（比例破
     * 了车会拐弯），motor 钳位是器件契约兜底。两层必须，且同引
     * ROBOT_MOTOR_SPEED_MAX_RPM 单一来源 */
    float worst = 0.0f;
    for (int i = 0; i < CHASSIS_WHEEL_COUNT; i++)
    {
        float m = (rpm_out[i] < 0.0f) ? -rpm_out[i] : rpm_out[i];
        if (m > worst)
        {
            worst = m;
        }
    }
    const float max_rpm = ROBOT_MOTOR_SPEED_MAX_RPM;
    if (worst > max_rpm)
    {
        float scale = max_rpm / worst;
        for (int i = 0; i < CHASSIS_WHEEL_COUNT; i++)
        {
            rpm_out[i] *= scale;
        }
    }
}

void Chassis_MecanumFK(const float wheel_omega_radps[CHASSIS_WHEEL_COUNT],
                       Chassis_Vec_t *phys)
{
    if (wheel_omega_radps == NULL || phys == NULL)
    {
        return;
    }

    float fl = wheel_omega_radps[0];
    float fr = wheel_omega_radps[1];
    float rl = wheel_omega_radps[2];
    float rr = wheel_omega_radps[3];

    float r4 = ROBOT_APP_MECANUM_R_M * 0.25f;
    phys->vx = r4 * (fl + fr + rl + rr);
    phys->vy = r4 * (-fl + fr + rl - rr);
    phys->omega = (ROBOT_APP_MECANUM_R_M / (4.0f * ROBOT_APP_MECANUM_L_M))
                  * (-fl + fr - rl + rr);
}
