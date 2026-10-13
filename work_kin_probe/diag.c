/* 诊断：为什么当前姿态下 NORMAL 逆解完全失败 */
#include <stdio.h>
#include <math.h>
#include "kinematics.h"
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F / 180.0f)

int main(void)
{
    /* 复刻 kinematics.c 的常量合成 */
    float ux = ROBOT_ARM_UPPER_X_MM, uz = ROBOT_ARM_UPPER_Z_MM;
    float la = sqrtf(ux*ux + uz*uz);
    float ax = ROBOT_ARM_ROLLJ_X_MM + ROBOT_ARM_WRIST_X_MM;
    float az = ROBOT_ARM_ROLLJ_Z_MM + ROBOT_ARM_WRIST_Z_MM;
    float lb = sqrtf(ax*ax + az*az);
    float base_elev_a = atan2f(uz, ux) / D2R;
    float base_elev_b = atan2f(az, ax) / D2R;

    float r4x = ROBOT_ARM_TCP_X_MM - (ROBOT_ARM_SHOULDER_X_MM + ux + ax);
    float r4z = ROBOT_ARM_TCP_Z_MM - (ROBOT_ARM_SHOULDER_Z_MM + uz + az);

    printf("=== 合成常量 ===\n");
    printf("  |U| 大臂        = %.4f mm   基准仰角 %.4f deg\n", la, base_elev_a);
    printf("  |P| 小臂等效     = %.4f mm   基准仰角 %.4f deg\n", lb, base_elev_b);
    printf("  可伸最长        = %.4f mm\n", la + lb);
    printf("  R4 = TCP0 - 腕俯仰轴 = (%.4f, %.4f)  |R4| = %.4f mm\n", r4x, r4z, sqrtf(r4x*r4x+r4z*r4z));
    printf("  ROBOT_ARM_SING_MARGIN = %.4f\n\n", ROBOT_ARM_SING_MARGIN);

    /* 建模姿态下腕俯仰轴到肩的距离 */
    float wx0 = ROBOT_ARM_SHOULDER_X_MM + ux + ax;
    float wz0 = ROBOT_ARM_SHOULDER_Z_MM + uz + az;
    printf("=== 建模位置 ===\n");
    printf("  肩 = (%.3f, %.3f)\n", ROBOT_ARM_SHOULDER_X_MM, ROBOT_ARM_SHOULDER_Z_MM);
    printf("  腕俯仰轴位 = (%.3f, %.3f)\n", wx0, wz0);
    float dx0 = wx0 - ROBOT_ARM_SHOULDER_X_MM, dz0 = wz0 - ROBOT_ARM_SHOULDER_Z_MM;
    float r0 = sqrtf(dx0*dx0 + dz0*dz0);
    printf("  |肩→腕俯仰轴| = %.4f mm   (可伸最长 %.4f)  余量 %+.4f mm\n",
           r0, la + lb, (la + lb) - r0);
    float c0 = (r0*r0 - la*la - lb*lb) / (2.0f*la*lb);
    printf("  该处 cos(Δ) = %.6f   → 1-|cos| = %.6f   (需 > margin=%.4f)\n\n",
           c0, 1.0f - fabsf(c0), ROBOT_ARM_SING_MARGIN);
    printf("  ⇒ 建模姿态本身就是【完全伸直奇异位】：1-|cos| = %.6f 远小于 %.4f\n",
           1.0f - fabsf(c0), ROBOT_ARM_SING_MARGIN);

    /* FK 一致性 */
    float deg0[3] = {90.0f, 90.0f, 90.0f};
    Kin_Pose_t p;
    Kin_Fk(deg0, 0.0f, &p);
    printf("\n=== FK(90,90,90) 与配置 TCP0 的偏差 ===\n");
    printf("  FK   = (%.4f, %.4f)\n", p.x_mm, p.z_mm);
    printf("  TCP0 = (%.4f, %.4f)\n", ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM);
    printf("  偏差 = (%.4f, %.4f)  |偏差| = %.4f mm\n",
           p.x_mm-ROBOT_ARM_TCP_X_MM, p.z_mm-ROBOT_ARM_TCP_Z_MM,
           sqrtf(powf(p.x_mm-ROBOT_ARM_TCP_X_MM,2)+powf(p.z_mm-ROBOT_ARM_TCP_Z_MM,2)));

    /* 用 FK 实际位置反推 NORMAL 逆解可用性 */
    printf("\n=== Kin_IkNormal 在 FK 实际位置附近的可用性 ===\n");
    printf("  目标(相对FK点 dx,dz) mm      结果\n");
    float dirs[4][2] = {{0,-1},{-1,0},{0,1},{1,0}};
    const char* nm[4] = {"下", "后", "上", "前"};
    for (int k = 0; k < 4; k++)
    {
        float dx = dirs[k][0], dz = dirs[k][1];
        float dird = atan2f(dz, dx) / D2R;
        float last = -1e9f;
        for (float s = -100.0f; s <= 100.0f; s += 0.5f)
        {
            float tgt[2] = { p.x_mm + dx*s, p.z_mm + dz*s };
            float out[3];
            /* 用该方向的法向角：这里方向=运动方向 */
            if (Kin_IkNormal(tgt, dird, out)) last = s;
        }
        printf("  沿%s 可用位移范围末端 = %+.1f mm\n", nm[k], last);
    }

    /* 更实际：法向取当前法向角，找可用区间 */
    float nd = Kin_NormalDir(deg0);   /* 建模位法向 */
    printf("\n  建模位法向仰角 = %.3f°\n", nd);
    float ndx = cosf(nd*D2R), ndz = sinf(nd*D2R);
    printf("  沿该法向可用 s（相对 FK 点）：\n");
    float lo = 1e9f, hi = -1e9f;
    for (float s = -200.0f; s <= 200.0f; s += 0.5f)
    {
        float tgt[2] = { p.x_mm + ndx*s, p.z_mm + ndz*s };
        float out[3];
        if (Kin_IkNormal(tgt, nd, out)) { if (s < lo) lo = s; if (s > hi) hi = s; }
    }
    printf("    s ∈ [%.1f, %.1f] mm\n", lo, hi);
    return 0;
}
