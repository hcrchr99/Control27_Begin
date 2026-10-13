/* 标定 WRIST_X/Z + TOOL_X/Z，使得：
 *   (1) FK(90,90,90) 命中 ROBOT_ARM_TCP_X/Z_MM
 *   (2) 腕俯仰轴仍落在原位置（ROLLJ 常量不动 → 肘→腕俯仰仍是 95.2026mm）
 *   (3) 法向仰角仍为 NORMAL0 的 50.535°
 * 约束：净 R4 = (TCP0 - 腕俯仰轴) 已知且唯一，
 *       故 Ry(90°)·G = R4 是唯一确定解，WRIST 对 FK 的影响被吸收。
 * 这里只做数值确认。
 */
#include <stdio.h>
#include <math.h>
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)

static void fk0(float gx, float gz, float *ox, float *oz)
{
    float a3 = 0.0f, ca3=1.0f, sa3=0.0f;
    float wxx = ROBOT_ARM_WRIST_X_MM + gx*ca3 - gz*sa3;
    float wzz = ROBOT_ARM_WRIST_Z_MM + gx*sa3 + gz*ca3;
    float wy  = ROBOT_ARM_WRIST_Y_MM + ROBOT_ARM_TOOL_Y_MM;
    float ax = ROBOT_ARM_ROLL_AX_X, az = ROBOT_ARM_ROLL_AX_Z;
    float tx = wxx, tz = wzz;                      /* roll=0 */
    float bx = ROBOT_ARM_ROLLJ_X_MM + tx, bz = ROBOT_ARM_ROLLJ_Z_MM + tz;
    float lb = sqrtf(bx*bx+bz*bz), bb = atan2f(bz,bx);
    float la = sqrtf(ROBOT_ARM_UPPER_X_MM*ROBOT_ARM_UPPER_X_MM
                   + ROBOT_ARM_UPPER_Z_MM*ROBOT_ARM_UPPER_Z_MM);
    float be = atan2f(ROBOT_ARM_UPPER_Z_MM, ROBOT_ARM_UPPER_X_MM);
    *ox = ROBOT_ARM_SHOULDER_X_MM + la*cosf(be) + lb*cosf(bb);
    *oz = ROBOT_ARM_SHOULDER_Z_MM + la*sinf(be) + lb*sinf(bb);
    (void)wy;
}

int main(void)
{
    float wp_x = ROBOT_ARM_SHOULDER_X_MM + ROBOT_ARM_UPPER_X_MM
               + ROBOT_ARM_ROLLJ_X_MM + ROBOT_ARM_WRIST_X_MM;
    float wp_z = ROBOT_ARM_SHOULDER_Z_MM + ROBOT_ARM_UPPER_Z_MM
               + ROBOT_ARM_ROLLJ_Z_MM + ROBOT_ARM_WRIST_Z_MM;
    printf("=== 现状 ===\n");
    printf("  腕俯仰轴位 (%.3f, %.3f)\n", wp_x, wp_z);
    float r4x = ROBOT_ARM_TCP_X_MM - wp_x, r4z = ROBOT_ARM_TCP_Z_MM - wp_z;
    printf("  注释里的 R4 = TCP0 - 腕俯仰轴 = (%.4f, %.4f)  |R4|=%.4f\n",
           r4x, r4z, sqrtf(r4x*r4x+r4z*r4z));
    printf("  TOOL 现值 = (%.4f, %.4f)  |·|=%.4f\n",
           ROBOT_ARM_TOOL_X_MM, ROBOT_ARM_TOOL_Z_MM,
           sqrtf(ROBOT_ARM_TOOL_X_MM*ROBOT_ARM_TOOL_X_MM
                +ROBOT_ARM_TOOL_Z_MM*ROBOT_ARM_TOOL_Z_MM));
    printf("  注释说 |面内分量|→94.434，但 R4 需要 |·|=%.3f  ← 二者矛盾\n\n",
           sqrtf(r4x*r4x+r4z*r4z));

    float x, z;
    fk0(ROBOT_ARM_TOOL_X_MM, ROBOT_ARM_TOOL_Z_MM, &x, &z);
    printf("  FK(90,90,90) = (%.4f, %.4f)   TCP0 = (%.3f, %.3f)   误差 %.4f mm\n\n",
           x, z, ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM,
           sqrtf(powf(x-ROBOT_ARM_TCP_X_MM,2)+powf(z-ROBOT_ARM_TCP_Z_MM,2)));

    /* 解析解：eff_link(roll=0, el3=0) 产出 (GX, GZ)
     * Kin_Fk 加 (WRIST_X, WRIST_Z) 后必须等于 R4 → 只要 GX=R4X, GZ=R4Z
     * 但那样 WRIST 会叠加两次，所以取方案：WRIST 缩小、GX/GZ = R4 - WRIST */
    printf("=== 标定解 ===\n");
    float wscale = 1.0f;
    float wxx = ROBOT_ARM_WRIST_X_MM * wscale;
    float wzz = ROBOT_ARM_WRIST_Z_MM * wscale;
    float gx = r4x - wxx;
    float gz = r4z - wzz;
    printf("  WRIST_X = %.4f  WRIST_Z = %.4f   (原 %.4f / %.4f)\n",
           wxx, wzz, ROBOT_ARM_WRIST_X_MM, ROBOT_ARM_WRIST_Z_MM);
    printf("  TOOL_X  = %.4f  TOOL_Z  = %.4f   (原 %.4f / %.4f)\n",
           gx, gz, ROBOT_ARM_TOOL_X_MM, ROBOT_ARM_TOOL_Z_MM);
    printf("  |TOOL| = %.4f   |WRIST| = %.4f\n",
           sqrtf(gx*gx+gz*gz), sqrtf(wxx*wxx+wzz*wzz));

    fk0(gx, gz, &x, &z);
    printf("\n  FK(90,90,90) = (%.4f, %.4f)   TCP0 = (%.3f, %.3f)   误差 %.6f mm\n",
           x, z, ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM,
           sqrtf(powf(x-ROBOT_ARM_TCP_X_MM,2)+powf(z-ROBOT_ARM_TCP_Z_MM,2)));

    /* 法向仰角 */
    float ne = atan2f(ROBOT_ARM_NORMAL0_Z, ROBOT_ARM_NORMAL0_X)*R2D;
    printf("  NORMAL0 仰角 = %.4f°   (运动方向 dir_deg 用同一口径)\n", ne);

    /* 肩→腕俯仰轴 & 奇异裕度 */
    float fx = ROBOT_ARM_ROLLJ_X_MM + wxx, fz = ROBOT_ARM_ROLLJ_Z_MM + wzz;
    float lb = sqrtf(fx*fx+fz*fz);
    float la = sqrtf(ROBOT_ARM_UPPER_X_MM*ROBOT_ARM_UPPER_X_MM
                   + ROBOT_ARM_UPPER_Z_MM*ROBOT_ARM_UPPER_Z_MM);
    printf("  肘→腕俯仰轴 = %.4f mm   可伸最长 = %.4f mm\n", lb, la+lb);
    float c = (lb*lb - la*la - lb*lb)/(2*la*lb);
    (void)c;
    /* 建模位就是伸直位：r0 = lb */
    float r0 = lb;
    c = (r0*r0 - la*la - lb*lb)/(2*la*lb);
    printf("  建模位 1-|cos| = %.6f   阈值 %.4f\n", 1.0f-fabsf(c), ROBOT_ARM_SING_MARGIN);
    printf("  → 需要把 SING_MARGIN 降到 < %.6f 才能从建模位起步\n", 1.0f-fabsf(c));
    return 0;
}
