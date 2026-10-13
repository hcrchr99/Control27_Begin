/* 收尾检查：竖直下压 70mm 全程关节范围、限位、速率，并导出轨迹表 */
#include <stdio.h>
#include <math.h>
#include "kinematics.h"
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)

static const float kMin[5] = ROBOT_SERVO_LIMIT_MIN_DEG_LIST;
static const float kMax[5] = ROBOT_SERVO_LIMIT_MAX_DEG_LIST;

int main(void)
{
    float deg0[3] = {90.0f, 90.0f, 90.0f};
    Kin_Pose_t p;
    Kin_Fk(deg0, 0.0f, &p);
    float nd = atan2f(ROBOT_ARM_NORMAL0_Z, ROBOT_ARM_NORMAL0_X)*R2D;

    printf("=== 竖直下压 70mm：从建模位 (%.3f, %.3f) ===\n", p.x_mm, p.z_mm);
    printf("=== 法向仰角 %.4f°（运动方向 dir_deg = %.4f）===\n\n", nd, nd);

    int n = 71;
    float lo[3] = {1e9f,1e9f,1e9f}, hi[3] = {-1e9f,-1e9f,-1e9f};
    float prev[3]; float maxstep[3] = {0,0,0};
    int viol = 0;
    printf("   #  目标z(mm)   舵机角 θ1/θ2/θ4 (°)         爪心FK(x,z)        Σ_el\n");
    for (int i = 0; i < n; i++)
    {
        float z = p.z_mm - 70.0f * i / (n - 1.0f);
        float d[3];
        if (!Kin_IkNormal((float[2]){p.x_mm, z}, nd, d))
        { printf("   %2d  z=%.2f  逆解失败\n", i, z); viol++; continue; }
        float cc[2]; Kin_FkNormal(d, cc);
        float sig = (90-d[0]) + (90-d[1]) + (90-d[2]);
        if (i % 10 == 0 || i == n-1)
            printf("   %2d  %8.2f   %7.2f / %7.2f / %7.2f   (%8.3f,%8.3f)  %+8.2f\n",
                   i, z, d[0], d[1], d[2], cc[0], cc[1], sig);
        for (int j = 0; j < 3; j++)
        {
            if (d[j] < lo[j]) lo[j] = d[j];
            if (d[j] > hi[j]) hi[j] = d[j];
            if (d[j] < kMin[j] - 1e-3f || d[j] > kMax[j] + 1e-3f)
            { printf("      ** 关节%d = %.2f 越限 [%.0f, %.0f]\n", j, d[j], kMin[j], kMax[j]); viol++; }
        }
        if (i > 0) for (int j = 0; j < 3; j++)
        { float dd = fabsf(d[j]-prev[j]); if (dd > maxstep[j]) maxstep[j] = dd; }
        for (int j = 0; j < 3; j++) prev[j] = d[j];
    }

    printf("\n=== 关节范围（%d 点全程）===\n", n);
    const char* nm[3] = {"θ1 大臂", "θ2 小臂", "θ4 手腕"};
    for (int j = 0; j < 3; j++)
        printf("  %s  舵机角 %.2f° ~ %.2f°   行程 %.2f°   限位[%.0f,%.0f]  %s\n",
               nm[j], lo[j], hi[j], hi[j]-lo[j], kMin[j], kMax[j],
               (lo[j]>=kMin[j]-1e-3f && hi[j]<=kMax[j]+1e-3f) ? "OK" : "**越限**");
    printf("\n  单步最大关节增量（相邻 1mm）： θ1 %.3f°  θ2 %.3f°  θ4 %.3f°\n",
           maxstep[0], maxstep[1], maxstep[2]);

    float seg_ms = 1.0f / ROBOT_ARM_LINE_SPEED_MMPS * 1000.0f;
    float allow = ROBOT_APP_JOINT_RATE_DPS * seg_ms * 0.001f;
    printf("  1mm 步对应 %.3f ms，速率上限 %.0f°/s → 允许 %.3f°/mm\n",
           seg_ms, ROBOT_APP_JOINT_RATE_DPS, allow);
    printf("  峰值速率需求： θ1 %.1f°/s  θ2 %.1f°/s  θ4 %.1f°/s\n",
           maxstep[0]/seg_ms*1000.0f, maxstep[1]/seg_ms*1000.0f, maxstep[2]/seg_ms*1000.0f);

    printf("\n  违规点 = %d\n", viol);

    /* 直线度 & 朝向 */
    printf("\n=== 直线度 / 朝向 ===\n");
    float xref = 0, zref = 0, sigref = 0;
    float maxlat = 0, maxang = 0;
    for (int i = 0; i < n; i++)
    {
        float z = p.z_mm - 70.0f*i/(n-1.0f);
        float d[3]; Kin_IkNormal((float[2]){p.x_mm, z}, nd, d);
        float cc[2]; Kin_FkNormal(d, cc);
        if (i == 0) { xref = cc[0]; zref = cc[1]; sigref = (90-d[0])+(90-d[1])+(90-d[2]); }
        float lat = fabsf(cc[0] - xref);
        if (lat > maxlat) maxlat = lat;
        float sig = (90-d[0])+(90-d[1])+(90-d[2]);
        if (fabsf(sig-sigref) > maxang) maxang = fabsf(sig-sigref);
    }
    printf("  横向（x 方向）最大偏差 = %.6f mm\n", maxlat);
    printf("  Σ 最大漂移 = %.6f°（= 朝向偏差）\n", maxang);
    printf("  实际下压深度 = %.3f mm\n", zref - p.z_mm + 70.0f - (zref - p.z_mm));
    return 0;
}
