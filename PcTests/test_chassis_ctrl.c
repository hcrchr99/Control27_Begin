/**
 * @file    test_chassis_ctrl.c
 * @brief   底盘业务链 PC 单测：手感整形 / 量纲换算 / 麦轮正逆解往返 / 饱和
 *
 * 纯函数直测（无替身）；输入即 rc_cmd 归一化快照量（按键适配在发送端，
 * 车载链不消费 keys——2026-10-11 裁决）。
 */
#include "test_util.h"
#include "chassis_ctrl.h"

void TestChassisCtrl_Run(void)
{
    printf("-- chassis_ctrl --\n");

    /* ===== Shape：死区 / 平方曲线 / 端点（期望值按宏现算，调参不破测试） ===== */
    {
        Chassis_Vec_t in, out;
        const double d = ROBOT_APP_CHASSIS_DEADBAND;

        in.vx = 0.03f; in.vy = 0.0f; in.omega = 0.0f;  /* 死区内（d<0.46 时必真） */
        Chassis_Shape(&in, &out);
        CHECK_NEAR(out.vx, 0.0, 1e-6);

        in.vx = 0.5f;                                  /* 死区外：((0.5-d)/(1-d))² */
        Chassis_Shape(&in, &out);
        CHECK_NEAR(out.vx, (0.5 - d) / (1.0 - d) * ((0.5 - d) / (1.0 - d)), 1e-4);
        CHECK(out.vx > 0.0f);

        in.vx = -0.5f;
        Chassis_Shape(&in, &out);
        CHECK_NEAR(out.vx, -(0.5 - d) / (1.0 - d) * ((0.5 - d) / (1.0 - d)), 1e-4);

        in.vx = 1.0f;
        Chassis_Shape(&in, &out);
        CHECK_NEAR(out.vx, 1.0, 1e-6);                 /* 满幅映射满幅 */

        in.vx = 1.7f;                                  /* 超幅钳位 */
        Chassis_Shape(&in, &out);
        CHECK_NEAR(out.vx, 1.0, 1e-6);
    }

    /* ===== ToPhysical：满幅换算 ===== */
    {
        Chassis_Vec_t n, p;
        n.vx = 1.0f; n.vy = -1.0f; n.omega = 1.0f;
        Chassis_ToPhysical(&n, &p);
        CHECK_NEAR(p.vx, ROBOT_APP_VX_MAX_MPS, 1e-6);
        CHECK_NEAR(p.vy, -ROBOT_APP_VY_MAX_MPS, 1e-6);
        CHECK_NEAR(p.omega, ROBOT_APP_W_MAX_RADPS, 1e-6);
    }

    /* ===== IK：三种基元方向 ===== */
    {
        Chassis_Vec_t p;
        float rpm[CHASSIS_WHEEL_COUNT];
        float w_full = ROBOT_APP_VX_MAX_MPS / ROBOT_APP_MECANUM_R_M
                       * 9.5492965855f;                /* 满幅平移的单轮 rpm */

        p.vx = ROBOT_APP_VX_MAX_MPS; p.vy = 0.0f; p.omega = 0.0f;
        Chassis_MecanumIK(&p, rpm);
        CHECK_NEAR(rpm[0], w_full, 1e-3);              /* 前进：四轮全正等速 */
        CHECK_NEAR(rpm[1], w_full, 1e-3);
        CHECK_NEAR(rpm[2], w_full, 1e-3);
        CHECK_NEAR(rpm[3], w_full, 1e-3);

        p.vx = 0.0f; p.vy = ROBOT_APP_VY_MAX_MPS; p.omega = 0.0f;
        Chassis_MecanumIK(&p, rpm);
        CHECK(rpm[0] < 0.0f);                          /* 左移：FL-/FR+/RL+/RR- */
        CHECK(rpm[1] > 0.0f);
        CHECK(rpm[2] > 0.0f);
        CHECK(rpm[3] < 0.0f);
        CHECK_NEAR(rpm[0], -rpm[1], 1e-3);

        p.vx = 0.0f; p.vy = 0.0f; p.omega = ROBOT_APP_W_MAX_RADPS;
        Chassis_MecanumIK(&p, rpm);
        CHECK(rpm[0] < 0.0f);                          /* 左转（CCW）：FL-/FR+/RL-/RR+ */
        CHECK(rpm[1] > 0.0f);
        CHECK(rpm[2] < 0.0f);
        CHECK(rpm[3] > 0.0f);
    }

    /* ===== IK+FK 往返恒等（无饱和段） ===== */
    {
        Chassis_Vec_t p, back;
        float rpm[CHASSIS_WHEEL_COUNT];
        float omega_radps[CHASSIS_WHEEL_COUNT];
        p.vx = 0.2f; p.vy = -0.1f; p.omega = 0.5f;
        Chassis_MecanumIK(&p, rpm);
        for (int i = 0; i < CHASSIS_WHEEL_COUNT; i++)
        {
            omega_radps[i] = rpm[i] * 0.10471975512f;  /* rpm → rad/s */
        }
        Chassis_MecanumFK(omega_radps, &back);
        CHECK_NEAR(back.vx, p.vx, 1e-4);
        CHECK_NEAR(back.vy, p.vy, 1e-4);
        CHECK_NEAR(back.omega, p.omega, 1e-4);
    }

    /* ===== IK：饱和等比缩放 ===== */
    {
        Chassis_Vec_t p;
        float rpm[CHASSIS_WHEEL_COUNT];
        p.vx = 1.0f; p.vy = 1.0f; p.omega = 1.0f;      /* 合成远超单轮上限 */
        Chassis_MecanumIK(&p, rpm);
        float worst = 0.0f;
        for (int i = 0; i < CHASSIS_WHEEL_COUNT; i++)
        {
            float m = (rpm[i] < 0.0f) ? -rpm[i] : rpm[i];
            if (m > worst)
            {
                worst = m;
            }
            CHECK(m <= ROBOT_MOTOR_SPEED_MAX_RPM + 1e-3f);  /* 全员不超速 */
        }
        CHECK_NEAR(worst, ROBOT_MOTOR_SPEED_MAX_RPM, 1e-2); /* 恰好顶满上限 */
        CHECK(rpm[0] < 0.0f && rpm[1] > 0.0f);         /* 方向没被缩没 */
    }
}
