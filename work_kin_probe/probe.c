/* 用真实代码核验 FK/IK：与数字孪生实测对比 */
#include <stdio.h>
#include <math.h>
#include "kinematics.h"
#include "robot_config.h"

int main(void)
{
    printf("=== 常量 ===\n");
    printf("SHOULDER (%.3f, %.3f)\n", ROBOT_ARM_SHOULDER_X_MM, ROBOT_ARM_SHOULDER_Z_MM);
    printf("UPPER    (%.3f, %.3f)\n", ROBOT_ARM_UPPER_X_MM, ROBOT_ARM_UPPER_Z_MM);
    printf("ROLLJ    (%.3f, %.3f, y=%.3f)\n", ROBOT_ARM_ROLLJ_X_MM, ROBOT_ARM_ROLLJ_Z_MM, ROBOT_ARM_ROLLJ_Y_MM);
    printf("WRIST    (%.3f, %.3f, y=%.3f)\n", ROBOT_ARM_WRIST_X_MM, ROBOT_ARM_WRIST_Z_MM, ROBOT_ARM_WRIST_Y_MM);
    printf("TOOL     (%.3f, %.3f, y=%.3f)\n", ROBOT_ARM_TOOL_X_MM, ROBOT_ARM_TOOL_Z_MM, ROBOT_ARM_TOOL_Y_MM);
    printf("TCP0     (%.3f, %.3f)\n", ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM);
    printf("NORMAL0  (%.4f, %.4f)\n", ROBOT_ARM_NORMAL0_X, ROBOT_ARM_NORMAL0_Z);
    printf("ROLL_AX  (%.6f, %.6f)\n", ROBOT_ARM_ROLL_AX_X, ROBOT_ARM_ROLL_AX_Z);
    {
        static const float kMin[5] = ROBOT_SERVO_LIMIT_MIN_DEG_LIST;
        static const float kMax[5] = ROBOT_SERVO_LIMIT_MAX_DEG_LIST;
        printf("LIMITS servo min = {%.0f,%.0f,%.0f,%.0f,%.0f}\n",
               kMin[0], kMin[1], kMin[2], kMin[3], kMin[4]);
        printf("LIMITS servo max = {%.0f,%.0f,%.0f,%.0f,%.0f}\n\n",
               kMax[0], kMax[1], kMax[2], kMax[3], kMax[4]);
    }

    /* 1) 零位（建模姿态）：三关节舵机角都该是 90° */
    float deg0[3] = { 90.0f, 90.0f, 90.0f };
    Kin_Pose_t p;
    Kin_Fk(deg0, 0.0f, &p);
    printf("=== FK(90,90,90), roll=0 ===\n");
    printf("  爪尖 = (%.3f, %.3f) mm   y=%.3f\n", p.x_mm, p.z_mm, p.y_mm);
    printf("  期望 TCP0 = (%.3f, %.3f) mm   y=%.3f\n", ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM, ROBOT_ARM_TOOL_Y_MM);
    printf("  偏差 = (%.3f, %.3f) mm\n\n", p.x_mm - ROBOT_ARM_TCP_X_MM, p.z_mm - ROBOT_ARM_TCP_Z_MM);

    /* 2) 截图姿态：大臂 104° / 小臂 101° / 手腕 55° / 爪旋转 176° */
    float degS[3] = { 104.0f, 101.0f, 55.0f };
    Kin_Fk(degS, 176.0f, &p);
    printf("=== FK(104,101,55), roll=176 (截图姿态) ===\n");
    printf("  爪尖 = (%.3f, %.3f) mm   y=%.3f\n", p.x_mm, p.z_mm, p.y_mm);
    printf("  开合方向投影 = (%.4f, %.4f)\n", p.open_x, p.open_z);
    float d = Kin_NormalDir(degS);
    printf("  爪板法向仰角 = %.3f°  (运动方向)\n\n", d);

    /* 3) NORMAL 逆解：沿该法向走 70mm */
    printf("=== NORMAL 逆解：沿法向(%.2f°) 从当前爪尖走 70mm ===\n", d);
    float dx = cosf(d * 3.14159265f / 180.0f), dz = sinf(d * 3.14159265f / 180.0f);
    printf("  方向单位向量 = (%.5f, %.5f)\n", dx, dz);
    printf("   步  目标(x,z) mm          逆解舵机角(°)            反向FK核验误差\n");
    for (int i = 0; i <= 7; i++)
    {
        float s = 70.0f * i / 7.0f;
        float tgt[2] = { p.x_mm + dx * s, p.z_mm + dz * s };
        float out[3];
        bool ok = Kin_IkNormal(tgt, d, out);
        if (!ok)
        {
            printf("   %2d  (%.2f, %.2f)   <== 逆解失败\n", i, tgt[0], tgt[1]);
            continue;
        }
        float cc[2];
        Kin_FkNormal(out, cc);
        printf("   %2d  (%8.2f, %8.2f)   %.2f / %.2f / %.2f      (%.4f, %.4f)\n",
               i, tgt[0], tgt[1], out[0], out[1], out[2], cc[0] - tgt[0], cc[1] - tgt[1]);
    }

    /* 4) 极限测试：沿法向最多能走多远 */
    printf("\n=== 沿法向最大行程扫描 ===\n");
    float last_ok = 0.0f;
    for (float s = 0.0f; s <= 200.0f; s += 1.0f)
    {
        float tgt[2] = { p.x_mm + dx * s, p.z_mm + dz * s };
        float out[3];
        if (Kin_IkNormal(tgt, d, out)) { last_ok = s; }
    }
    printf("  当前姿态下沿该法向最大可走 %.1f mm（再远逆解失败）\n", last_ok);

    /* 5) 竖直向下（-90°）能走多远 */
    printf("\n=== 竖直向下(-90°)最大行程扫描（起点用当前爪尖）===\n");
    last_ok = 0.0f;
    for (float s = 0.0f; s <= 250.0f; s += 1.0f)
    {
        float tgt[2] = { p.x_mm, p.z_mm - s };
        float out[3];
        if (Kin_IkNormal(tgt, -90.0f, out)) { last_ok = s; }
    }
    printf("  竖直向下最大 %.1f mm\n", last_ok);
    return 0;
}
