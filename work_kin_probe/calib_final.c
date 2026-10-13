/* 极坐标标定 TOOL：搜索 (|G|, 安装角) 使 FK(90,90,90) 命中 TCP0。
 * 复刻 Kin_Fk 的口径（已由 cross.c 与真实 Kin_Fk 交叉验证一致）。 */
#include <stdio.h>
#include <math.h>
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)

static float GX, GZ;

/* 与 kinematics.c 的 Kin_Fk(deg=90,90,90), roll=0 完全同口径 */
static void fk0(float *ox, float *oz, float *oy)
{
    float el3 = 0.0f;                       /* servo 90 → el 0 */
    float a3 = el3 * D2R;
    float ca3 = cosf(a3), sa3 = sinf(a3);
    float wxx = ROBOT_ARM_WRIST_X_MM + GX*ca3 - GZ*sa3;
    float wzz = ROBOT_ARM_WRIST_Z_MM + GX*sa3 + GZ*ca3;
    float wy  = ROBOT_ARM_WRIST_Y_MM + ROBOT_ARM_TOOL_Y_MM;
    float ax = ROBOT_ARM_ROLL_AX_X, az = ROBOT_ARM_ROLL_AX_Z;
    float px = -az, pz = ax;
    float adotw = ax*wxx + az*wzz;
    float cr = 1.0f, sr = 0.0f;             /* roll = 0 */
    float tx = cr*wxx + sr*wy*px + (1.0f-cr)*adotw*ax;
    float tz = cr*wzz + sr*wy*pz + (1.0f-cr)*adotw*az;
    float bx = ROBOT_ARM_ROLLJ_X_MM + tx;
    float bz = ROBOT_ARM_ROLLJ_Z_MM + tz;
    float lb = sqrtf(bx*bx+bz*bz), bb = atan2f(bz,bx);
    float la = sqrtf(ROBOT_ARM_UPPER_X_MM*ROBOT_ARM_UPPER_X_MM
                   + ROBOT_ARM_UPPER_Z_MM*ROBOT_ARM_UPPER_Z_MM);
    float be = atan2f(ROBOT_ARM_UPPER_Z_MM, ROBOT_ARM_UPPER_X_MM);
    *ox = ROBOT_ARM_SHOULDER_X_MM + la*cosf(be) + lb*cosf(bb);
    *oz = ROBOT_ARM_SHOULDER_Z_MM + la*sinf(be) + lb*sinf(bb);
    *oy = ROBOT_ARM_ROLLJ_Y_MM + wy*cr + (az*wxx-ax*wzz)*sr;
}

static float err2(void)
{
    float x,z,y; fk0(&x,&z,&y);
    float dx = x-ROBOT_ARM_TCP_X_MM, dz = z-ROBOT_ARM_TCP_Z_MM;
    return dx*dx+dz*dz;
}

int main(void)
{
    float x,z,y;
    GX = ROBOT_ARM_TOOL_X_MM; GZ = ROBOT_ARM_TOOL_Z_MM;
    fk0(&x,&z,&y);
    printf("=== 现状 ===\n");
    printf("  TOOL = (%.4f, %.4f)  |G|=%.4f 角=%.4f°\n", GX, GZ,
           sqrtf(GX*GX+GZ*GZ), atan2f(GZ,GX)*R2D);
    printf("  FK(90,90,90) = (%.4f, %.4f) y=%.4f   TCP0=(%.3f, %.3f)  误差 %.4f mm\n\n",
           x,z,y, ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM, sqrtf(err2()));

    float g0 = sqrtf(GX*GX+GZ*GZ), a0 = atan2f(GZ,GX)*R2D;
    float best_g = g0, best_a = a0, best = err2();
    printf("=== 网格 + 精化搜索 (|G|, 安装角) ===\n");
    for (int pass = 0; pass < 6; pass++)
    {
        float gs = (pass == 0) ? 1.0f  : 0.5f;
        float as = (pass == 0) ? 1.0f  : 0.5f;
        float gstep = (pass == 0) ? 2.0f : 0.5f * powf(0.35f, (float)pass);
        float astep = (pass == 0) ? 2.0f : 0.5f * powf(0.35f, (float)pass);
        if (pass >= 2) { gstep = 0.02f*powf(0.4f,(float)(pass-2)); astep = 0.02f*powf(0.4f,(float)(pass-2)); }
        (void)gs; (void)as;
        for (float dg = -3.0f; dg <= 3.001f; dg += gstep)
        for (float da = -3.0f; da <= 3.001f; da += astep)
        {
            float g = best_g + dg, a = best_a + da;
            if (g < 40.0f || g > 160.0f) continue;
            GX = g*cosf(a*D2R); GZ = g*sinf(a*D2R);
            float e = err2();
            if (e < best) { best = e; }
        }
        /* 把最优写回（需再扫一遍找 argmin） */
        float bg = best_g, ba = best_a; float be = best;
        for (float dg = -3.0f; dg <= 3.001f; dg += gstep)
        for (float da = -3.0f; da <= 3.001f; da += astep)
        {
            float g = best_g + dg, a = best_a + da;
            if (g < 40.0f || g > 160.0f) continue;
            GX = g*cosf(a*D2R); GZ = g*sinf(a*D2R);
            float e = err2();
            if (e <= be + 1e-12f) { be = e; bg = g; ba = a; }
        }
        best_g = bg; best_a = ba; best = be;
    }
    GX = best_g*cosf(best_a*D2R); GZ = best_g*sinf(best_a*D2R);
    fk0(&x,&z,&y);
    printf("  最优 |G| = %.6f mm   安装角 = %.6f°\n", best_g, best_a);
    printf("  ⇒ TOOL_X_MM = %.4f   TOOL_Z_MM = %.4f\n", GX, GZ);
    printf("  FK(90,90,90) = (%.6f, %.6f) y=%.4f   误差 %.6f mm\n", x, z, y, sqrtf(best));
    printf("  （原 |G|=%.4f 角=%.4f°）\n\n", g0, a0);

    /* 检查法向一致性：NORMAL0 仰角应等于 G+90° 的口径 */
    float norm_elev = atan2f(ROBOT_ARM_NORMAL0_Z, ROBOT_ARM_NORMAL0_X)*R2D;
    printf("=== 法向一致性 ===\n");
    printf("  NORMAL0 仰角 = %.4f°\n", norm_elev);
    printf("  标定后爪安装角 = %.4f° → 对应法向仰角 %.4f°\n",
           best_a, best_a + 90.0f);
    printf("  差 = %.4f°\n", fabsf((best_a + 90.0f) - norm_elev));
    return 0;
}
