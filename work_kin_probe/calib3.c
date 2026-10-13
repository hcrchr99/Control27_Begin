/* 正确标定：保持 TOOL 方向（法向仰角不变），缩放 |TOOL| 使 FK(90,90,90) 命中 TCP0 */
#include <stdio.h>
#include <math.h>
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)

static float GX, GZ;

static void fk0(float *ox, float *oz)
{
    float ca3=1.0f, sa3=0.0f;                 /* el3 = 0 */
    float wxx = ROBOT_ARM_WRIST_X_MM + GX*ca3 - GZ*sa3;
    float wzz = ROBOT_ARM_WRIST_Z_MM + GX*sa3 + GZ*ca3;
    float bx = ROBOT_ARM_ROLLJ_X_MM + wxx, bz = ROBOT_ARM_ROLLJ_Z_MM + wzz;
    float lb = sqrtf(bx*bx+bz*bz), bb = atan2f(bz,bx);
    float la = sqrtf(ROBOT_ARM_UPPER_X_MM*ROBOT_ARM_UPPER_X_MM
                   + ROBOT_ARM_UPPER_Z_MM*ROBOT_ARM_UPPER_Z_MM);
    float be = atan2f(ROBOT_ARM_UPPER_Z_MM, ROBOT_ARM_UPPER_X_MM);
    *ox = ROBOT_ARM_SHOULDER_X_MM + la*cosf(be) + lb*cosf(bb);
    *oz = ROBOT_ARM_SHOULDER_Z_MM + la*sinf(be) + lb*sinf(bb);
}

int main(void)
{
    float x0, z0;
    fk0(&x0, &z0);
    printf("=== 现状 ===\n");
    printf("  TOOL = (%.4f, %.4f)  |TOOL| = %.4f  仰角 %.4f°\n",
           ROBOT_ARM_TOOL_X_MM, ROBOT_ARM_TOOL_Z_MM,
           sqrtf(ROBOT_ARM_TOOL_X_MM*ROBOT_ARM_TOOL_X_MM
                +ROBOT_ARM_TOOL_Z_MM*ROBOT_ARM_TOOL_Z_MM),
           atan2f(ROBOT_ARM_TOOL_Z_MM, ROBOT_ARM_TOOL_X_MM)*R2D);
    printf("  FK(90,90,90) = (%.4f, %.4f)   TCP0 = (%.3f, %.3f)   误差 %.4f mm\n\n",
           x0, z0, ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM,
           sqrtf(powf(x0-ROBOT_ARM_TCP_X_MM,2)+powf(z0-ROBOT_ARM_TCP_Z_MM,2)));

    /* 需要的位移量 */
    float need_x = ROBOT_ARM_TCP_X_MM - x0;
    float need_z = ROBOT_ARM_TCP_Z_MM - z0;
    printf("  需要补给 (%.4f, %.4f) mm   |·| = %.4f mm\n",
           need_x, need_z, sqrtf(need_x*need_x+need_z*need_z));

    /* 保持方向缩放：eff_link 输出位移 = G（roll=0,el3=0），故直接加 need */
    GX = ROBOT_ARM_TOOL_X_MM; GZ = ROBOT_ARM_TOOL_Z_MM;
    float gx = GX + need_x, gz = GZ + need_z;
    float x1, z1;
    GX = gx; GZ = gz;
    fk0(&x1, &z1);
    printf("\n=== 平移式修正（不建议：改了安装角）===\n");
    printf("  TOOL = (%.4f, %.4f)  |TOOL| = %.4f  仰角 %.4f°\n",
           gx, gz, sqrtf(gx*gx+gz*gz), atan2f(gz,gx)*R2D);
    printf("  FK = (%.4f, %.4f)  误差 %.6f mm\n", x1, z1,
           sqrtf(powf(x1-ROBOT_ARM_TCP_X_MM,2)+powf(z1-ROBOT_ARM_TCP_Z_MM,2)));

    /* 等比缩放：二分求 s 使 |FK(s) - TCP0| = 0（误差随 s 单调） */
    float s = 1.0f;
    {
        float lo = 0.5f, hi = 1.5f;
        for (int it = 0; it < 100; it++)
        {
            s = 0.5f * (lo + hi);
            GX = ROBOT_ARM_TOOL_X_MM * s; GZ = ROBOT_ARM_TOOL_Z_MM * s;
            float xx, zz; fk0(&xx, &zz);
            /* 用 x 分量判单调（x 随 s 单调增） */
            if (xx < ROBOT_ARM_TCP_X_MM) lo = s; else hi = s;
        }
    }
    GX = ROBOT_ARM_TOOL_X_MM * s; GZ = ROBOT_ARM_TOOL_Z_MM * s;
    {
        float xx, zz; fk0(&xx, &zz);
        printf("\n=== 等比缩放解（保持安装角）===\n");
        printf("  缩放系数 s = %.6f\n", s);
        printf("  TOOL_X_MM = %.4f   TOOL_Z_MM = %.4f\n", GX, GZ);
        printf("  |TOOL| = %.4f  仰角 %.4f°  (原 %.4f / %.4f°)\n",
               sqrtf(GX*GX+GZ*GZ), atan2f(GZ,GX)*R2D,
               sqrtf(ROBOT_ARM_TOOL_X_MM*ROBOT_ARM_TOOL_X_MM
                    +ROBOT_ARM_TOOL_Z_MM*ROBOT_ARM_TOOL_Z_MM),
               atan2f(ROBOT_ARM_TOOL_Z_MM, ROBOT_ARM_TOOL_X_MM)*R2D);
        printf("  FK(90,90,90) = (%.4f, %.4f)   TCP0 = (%.3f, %.3f)   误差 %.6f mm\n",
               xx, zz, ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM,
               sqrtf(powf(xx-ROBOT_ARM_TCP_X_MM,2)+powf(zz-ROBOT_ARM_TCP_Z_MM,2)));
    }
    return 0;
}
