/* 按 Kin_LineStart 的真实口径复测：i=0 允许失败，从 i=1 起计速率 */
#include <stdio.h>
#include <math.h>
#include "kinematics.h"
#include "robot_config.h"
#define PI_F 3.14159265f
#define R2D (180.0f/PI_F)

int main(void)
{
    float deg0[3] = {90.0f, 90.0f, 90.0f};
    Kin_Pose_t p; Kin_Fk(deg0, 0.0f, &p);
    float nd = atan2f(ROBOT_ARM_NORMAL0_Z, ROBOT_ARM_NORMAL0_X)*R2D;

    const float len = 70.0f;
    int n = (int)(len / ROBOT_ARM_LINE_STEP_MM) + 1;
    float seg_mm = len / (float)n;
    float seg_ms = seg_mm / ROBOT_ARM_LINE_SPEED_MMPS * 1000.0f;
    float max_step_deg = ROBOT_APP_JOINT_RATE_DPS * seg_ms * 0.001f;

    printf("=== Kin_LineStart 口径复测 ===\n");
    printf("  线长 %.1f mm  步长 %.1f mm → n=%d  实测段长 %.4f mm\n",
           len, ROBOT_ARM_LINE_STEP_MM, n, seg_mm);
    printf("  段时长 %.4f ms  允许每步关节变化 ≤ %.4f°\n\n", seg_ms, max_step_deg);

    float prev[3] = {0,0,0}; bool prev_valid = false;
    int fails = 0; int rate_viol = 0;
    float peak[3] = {0,0,0};
    printf("    i   目标z(mm)    θ1      θ2      θ4      Δθ1     Δθ2     Δθ4\n");
    for (int i = 0; i <= n; i++)
    {
        float t = (float)i/(float)n;
        float z = p.z_mm - len*t;
        float d[3];
        bool ok = Kin_IkNormal((float[2]){p.x_mm, z}, nd, d);
        if (!ok)
        {
            if (i == 0) { printf("     0  z=%.2f   （起点奇异，代码设计允许跳过）\n", z); continue; }
            printf("   %3d  z=%.2f   ** 逆解失败 → 整条拒绝 **\n", i, z);
            fails++;
            break;
        }
        if (prev_valid)
        {
            float dv[3];
            for (int j=0;j<3;j++){ dv[j]=fabsf(d[j]-prev[j]); if(dv[j]>peak[j])peak[j]=dv[j]; }
            bool bad = (dv[0]>max_step_deg+0.01f)||(dv[1]>max_step_deg+0.01f)||(dv[2]>max_step_deg+0.01f);
            if (bad) rate_viol++;
            if (i<=3 || i%4==0 || bad || i==n)
                printf("   %3d  %8.2f  %7.2f %7.2f %7.2f  %6.3f  %6.3f  %6.3f%s\n",
                       i, z, d[0],d[1],d[2], dv[0],dv[1],dv[2], bad?"  **超速**":"");
        }
        for(int j=0;j<3;j++) prev[j]=d[j];
        prev_valid = true;
    }
    printf("\n  中途逆解失败 = %d   速率超限步数 = %d\n", fails, rate_viol);
    printf("  峰值每步增量： Δθ1 %.4f°  Δθ2 %.4f°  Δθ4 %.4f°   （上限 %.4f°）\n",
           peak[0], peak[1], peak[2], max_step_deg);
    printf("  峰值关节速率： θ1 %.2f°/s  θ2 %.2f°/s  θ4 %.2f°/s   （上限 %.0f°/s）\n",
           peak[0]/seg_ms*1000.0f, peak[1]/seg_ms*1000.0f, peak[2]/seg_ms*1000.0f,
           ROBOT_APP_JOINT_RATE_DPS);
    printf("\n  ⇒ Kin_LineStart 判定：%s\n",
           (fails==0 && rate_viol==0) ? "**通过**" : "**拒绝**");
    return 0;
}
