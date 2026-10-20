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
    float t[3] = { 0.0f, 0.0f, 0.0f };  /* 死区重铺后的线性行程 0..1（符号不在这里；死区轴恒 0） */
    float shaped[3] = { 0.0f, 0.0f, 0.0f };
    int dom = -1;               /* 行程最大的轴（手感曲线的"主轴"） */

    /* 第一遍：逐轴死区+重铺量程（t=(mag-d)/(1-d)，先不平方） */
    for (int i = 0; i < 3; i++)
    {
        float x = ClampSym_(axes[i], 1.0f);
        float mag = (x < 0.0f) ? -x : x;

        if (mag > d)
        {
            t[i] = (mag - d) / (1.0f - d);
            if ((dom < 0) || (t[i] > t[dom]))
            {
                dom = i;
            }
        }
    }

    if (dom < 0)
    {
        out->vx = 0.0f;         /* 三轴全在死区：全零 */
        out->vy = 0.0f;
        out->omega = 0.0f;
        return;
    }

    /* 第二遍：平方曲线只捏主轴（t_dom²），其余轴按主轴行程等比跟随（×t_dom）。
     * 单轴时 dom 就是那根轴，输出=t²，与逐轴平方逐位一致；组合输入（如单杆
     * 斜推=前进+回转）方向在整形后保持——交换率不再被"小值平方更小"放大，
     * 修的正是"斜一点前进先塌、旋转还没起"的失配（2026-10-20 拍板） */
    for (int i = 0; i < 3; i++)
    {
        if (t[i] > 0.0f)
        {
            float mag = t[i] * t[dom];      /* i==dom 时恰为 t² */
            shaped[i] = (axes[i] < 0.0f) ? -mag : mag;
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
