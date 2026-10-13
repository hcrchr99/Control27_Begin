/* 改完常量后的完整验证：
 *  1) FK(90,90,90) 是否命中 TCP0
 *  2) 建模位奇异裕度
 *  3) Kin_LineStart 全线预扫：竖直下压 70mm 能否通过（含关节速率校验）
 *  4) 用 Kin_LineStep 走完，逐点核对直线度与朝向
 */
#include <stdio.h>
#include <math.h>
#include "kinematics.h"
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)

int main(void)
{
    float deg0[3] = {90.0f, 90.0f, 90.0f};
    Kin_Pose_t p;
    Kin_Fk(deg0, 0.0f, &p);
    printf("=== 1) FK 自洽 ===\n");
    printf("  Kin_Fk(90,90,90) = (%.4f, %.4f)  y=%.4f\n", p.x_mm, p.z_mm, p.y_mm);
    printf("  ROBOT_ARM_TCP_X/Z_MM = (%.3f, %.3f)\n",
           ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM);
    printf("  误差 = %.6f mm\n\n",
           sqrtf(powf(p.x_mm-ROBOT_ARM_TCP_X_MM,2)+powf(p.z_mm-ROBOT_ARM_TCP_Z_MM,2)));

    /* 2) 奇异裕度 */
    float la = sqrtf(ROBOT_ARM_UPPER_X_MM*ROBOT_ARM_UPPER_X_MM
                   + ROBOT_ARM_UPPER_Z_MM*ROBOT_ARM_UPPER_Z_MM);
    float fx = ROBOT_ARM_ROLLJ_X_MM + ROBOT_ARM_WRIST_X_MM;
    float fz = ROBOT_ARM_ROLLJ_Z_MM + ROBOT_ARM_WRIST_Z_MM;
    float lb = sqrtf(fx*fx+fz*fz);
    float r0 = lb;
    float c = (r0*r0 - la*la - lb*lb)/(2*la*lb);
    printf("=== 2) 建模位奇异裕度 ===\n");
    printf("  |U|=%.4f |P|=%.4f  1-|cos| = %.6f  阈值 = %.4f  → %s\n\n",
           la, lb, 1.0f-fabsf(c), ROBOT_ARM_SING_MARGIN,
           (1.0f-fabsf(c)) > ROBOT_ARM_SING_MARGIN ? "通过" : "**被拒**");

    /* 3) 竖直下压 70mm 全线预扫 */
    float nd = atan2f(ROBOT_ARM_NORMAL0_Z, ROBOT_ARM_NORMAL0_X)*R2D;
    printf("=== 3) 竖直下压 70mm ===\n");
    printf("  NORMAL0 仰角 = %.4f°（法向=该值；向下运动方向 dir_deg = 该值）\n", nd);

    Kin_LineReq_t req;
    req.mode = KIN_MODE_NORMAL;
    req.from_x_mm = p.x_mm;  req.from_z_mm = p.z_mm;
    req.to_x_mm = p.x_mm;    req.to_z_mm = p.z_mm - 70.0f;
    req.tool_pitch_deg = 0; req.tool_pitch0_deg = 0;
    req.roll_deg = 0; req.wrist_deg = 90.0f;
    req.dir_deg = nd;                     /* 竖直向下：法向朝下 = −90 域外的口径 */

    Kin_Line_t line;
    uint32_t t = 0;
    bool ok = Kin_LineStart(&line, &req, t);
    printf("  Kin_LineStart(...) = %s\n", ok ? "true  **通过**" : "false 被拒");
    if (!ok)
    {
        /* 逐点诊断为什么被拒 */
        int n = (int)(70.0f / ROBOT_ARM_LINE_STEP_MM) + 1;
        printf("  逐点诊断（步长 %.1fmm，共 %d 点）：\n", ROBOT_ARM_LINE_STEP_MM, n);
        float seg_ms = (70.0f/n) / ROBOT_ARM_LINE_SPEED_MMPS * 1000.0f;
        float max_step = ROBOT_APP_JOINT_RATE_DPS * seg_ms * 0.001f;
        printf("    每步允许关节变化 ≤ %.3f°\n", max_step);
        float prev[3] = {0,0,0}; bool pv = false;
        for (int i = 0; i <= n; i++)
        {
            float tt = (float)i/(float)n;
            float x = req.from_x_mm;
            float z = req.from_z_mm + (req.to_z_mm-req.from_z_mm)*tt;
            float d[3];
            bool r = Kin_IkNormal((float[2]){x,z}, req.dir_deg, d);
            if (!r) { printf("    点%d (%.2f,%.2f) 逆解失败\n", i, x, z); }
            else {
                if (pv) {
                    float mx = 0; int jm=0;
                    for (int j=0;j<3;j++){ float dd=fabsf(d[j]-prev[j]); if(dd>mx){mx=dd;jm=j;} }
                    printf("    点%d (%.2f,%.2f) 角(%.2f,%.2f,%.2f) 最大步进 %.3f° @j%d%s\n",
                           i, x, z, d[0],d[1],d[2], mx, jm, mx>max_step+0.01f?"  **超速**":"");
                } else {
                    printf("    点%d (%.2f,%.2f) 角(%.2f,%.2f,%.2f)\n", i, x, z, d[0],d[1],d[2]);
                }
                for(int j=0;j<3;j++) prev[j]=d[j];
                pv = true;
            }
        }
        return 0;
    }

    /* 4) 走完并核对 */
    printf("\n=== 4) Kin_LineStep 逐拍走完（核对直线度 / 朝向）===\n");
    printf("   拍   目标(x,z) mm          舵机角(°)            爪心FK(x,z)          偏差\n");
    float cc0[2] = {0,0};
    bool first = true;
    int lastx = -1;
    for (uint32_t ms = 0; ms <= 20000; ms += 20)
    {
        float d[3];
        if (!Kin_LineStep(&line, ms, d)) continue;
        float cc[2]; Kin_FkNormal(d, cc);
        if (first) { cc0[0]=cc[0]; cc0[1]=cc[1]; first=false; }
        float prog = (cc0[1]-cc[1]);
        int key = (int)(prog/10.0f);
        if (key != lastx || !line.active)
        {
            lastx = key;
            printf("   %4u  (%.2f, %.2f)      (%.2f, %.2f, %.2f)   (%.3f, %.3f)   dx=%+.4f\n",
                   ms, req.from_x_mm, req.from_z_mm - prog, d[0], d[1], d[2],
                   cc[0], cc[1], cc[0]-cc0[0]);
        }
        if (!line.active) break;
    }
    return 0;
}
