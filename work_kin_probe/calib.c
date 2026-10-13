/* 数值标定：求 TOOL_X/TOOL_Z（保持其它常量）使 FK(90,90,90) 命中孪生实测 TCP0
 * eff_link 会被 Kin_Fk 使用，这里直接内联同一套公式做优化。
 */
#include <stdio.h>
#include <math.h>
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)

static float g_gx, g_gz;   /* 待求 */

/* 复刻 Kin_Fk(deg=90,90,90), roll=0 */
static void fk0(float *ox, float *oz)
{
    float el3 = 0.0f;                       /* servo 90 → el 0 */
    float a3 = el3 * D2R;
    float ca3 = cosf(a3), sa3 = sinf(a3);
    float wxx = ROBOT_ARM_WRIST_X_MM + g_gx*ca3 - g_gz*sa3;
    float wzz = ROBOT_ARM_WRIST_Z_MM + g_gx*sa3 + g_gz*ca3;
    float wy  = ROBOT_ARM_WRIST_Y_MM + ROBOT_ARM_TOOL_Y_MM;
    float ax = ROBOT_ARM_ROLL_AX_X, az = ROBOT_ARM_ROLL_AX_Z;
    float px = -az, pz = ax;
    float adotw = ax*wxx + az*wzz;
    float cr = 1.0f, sr = 0.0f;             /* roll = 0 */
    float tx = cr*wxx + sr*wy*px + (1.0f-cr)*adotw*ax;
    float tz = cr*wzz + sr*wy*pz + (1.0f-cr)*adotw*az;
    float bx = ROBOT_ARM_ROLLJ_X_MM + tx;
    float bz = ROBOT_ARM_ROLLJ_Z_MM + tz;
    float len_b = sqrtf(bx*bx+bz*bz), base_b = atan2f(bz,bx);
    float la = sqrtf(ROBOT_ARM_UPPER_X_MM*ROBOT_ARM_UPPER_X_MM
                   + ROBOT_ARM_UPPER_Z_MM*ROBOT_ARM_UPPER_Z_MM);
    float beta1 = atan2f(ROBOT_ARM_UPPER_Z_MM, ROBOT_ARM_UPPER_X_MM);
    /* el1=el2=0 → a1 = β1, a2 = base_b */
    *ox = ROBOT_ARM_SHOULDER_X_MM + la*cosf(beta1) + len_b*cosf(base_b);
    *oz = ROBOT_ARM_SHOULDER_Z_MM + la*sinf(beta1) + len_b*sinf(base_b);
}

static float cost(void)
{
    float x, z; fk0(&x, &z);
    float dx = x - ROBOT_ARM_TCP_X_MM, dz = z - ROBOT_ARM_TCP_Z_MM;
    return dx*dx + dz*dz;
}

int main(void)
{
    g_gx = ROBOT_ARM_TOOL_X_MM; g_gz = ROBOT_ARM_TOOL_Z_MM;
    float best = cost();
    printf("起点 TOOL=(%.4f, %.4f)  FK=(%%.4f, %%.4f)  cost=%.4f\n", g_gx, g_gz, best);
    {
        float x, z; fk0(&x, &z);
        printf("  起点 TOOL=(%.4f, %.4f)  FK=(%.4f, %.4f)  误差 %.4f mm\n",
               g_gx, g_gz, x, z, sqrtf(best));
    }
    /* 坐标下降 + 步长收缩 */
    float step = 20.0f;
    for (int pass = 0; pass < 200; pass++)
    {
        int improved = 0;
        for (int axis = 0; axis < 2; axis++)
        {
            for (int sgn = -1; sgn <= 1; sgn += 2)
            {
                float ox = g_gx, oz = g_gz;
                if (axis == 0) g_gx += sgn*step; else g_gz += sgn*step;
                float c = cost();
                if (c < best) { best = c; improved = 1; }
                else { g_gx = ox; g_gz = oz; }
            }
        }
        if (!improved) step *= 0.5f;
        if (step < 1e-6f) break;
    }
    {
        float x, z; fk0(&x, &z);
        printf("\n标定结果 TOOL_X_MM = %.4f   TOOL_Z_MM = %.4f\n", g_gx, g_gz);
        printf("  FK(90,90,90) = (%.4f, %.4f)   目标 (%.3f, %.3f)   残差 %.6f mm\n",
               x, z, ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM, sqrtf(best));
        printf("  |G| 原 %.4f → 新 %.4f\n",
               sqrtf(ROBOT_ARM_TOOL_X_MM*ROBOT_ARM_TOOL_X_MM
                     + ROBOT_ARM_TOOL_Z_MM*ROBOT_ARM_TOOL_Z_MM),
               sqrtf(g_gx*g_gx+g_gz*g_gz));
    }
    /* 报告：修正后建模位的可伸余量 & 奇异裕度 */
    {
        float ux=ROBOT_ARM_UPPER_X_MM, uz=ROBOT_ARM_UPPER_Z_MM;
        float la = sqrtf(ux*ux+uz*uz);
        float a3=0, ca3=1, sa3=0;
        float wxx = ROBOT_ARM_WRIST_X_MM + g_gx*ca3 - g_gz*sa3;
        float wzz = ROBOT_ARM_WRIST_Z_MM + g_gx*sa3 + g_gz*ca3;
        float tx=wxx, tz=wzz;
        float bx = ROBOT_ARM_ROLLJ_X_MM + tx, bz = ROBOT_ARM_ROLLJ_Z_MM + tz;
        float lb = sqrtf(bx*bx+bz*bz);
        printf("\n  修正后 |WRIST→TOOL| 等效 = %.4f   可伸最长 = %.4f\n", lb, la+lb);
        printf("\n  [note] lb = equivalent ROLLJ-center -> claw length, not shoulder->wrist\n");
        /* 肩→腕俯仰轴 */
        float fx = ROBOT_ARM_ROLLJ_X_MM + ROBOT_ARM_WRIST_X_MM;
        float fz = ROBOT_ARM_ROLLJ_Z_MM + ROBOT_ARM_WRIST_Z_MM;
        float r0 = sqrtf(fx*fx+fz*fz);
        printf("  肩→腕俯仰轴 = %.4f mm   可伸最长 %.4f   余量 %+.4f mm\n",
               r0, la+lb, (la+lb)-r0);
    }
    return 0;
}
