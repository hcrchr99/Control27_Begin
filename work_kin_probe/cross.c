/* 核对 C 的 Kin_IkNormal 与 Python claw_traj 求解出的关节角是否一致
 * 用已验证可行的竖直下压轨迹（Σ_el = 140.5°，用户 claw_traj 口径）
 */
#include <stdio.h>
#include <math.h>
#include "kinematics.h"
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F / 180.0f)
#define R2D (180.0f / PI_F)

int main(void)
{
    /* 用户 claw_traj.py 给出的竖直下压轨迹（θ1,θ2,θ4 单位度，Σ=140.5°）
     * 这里 θ 是 URDF/孪生口径；C 代码用舵机角 servo = 90 - el，el = -θ */
    float user[][3] = {
        { -15.961f, 82.772f, 73.688f },
        { -10.097f, 85.137f, 65.460f },
        {  -3.384f, 85.605f, 58.279f },
        {  +3.978f, 84.181f, 52.341f },
        {  +7.848f, 82.756f, 49.896f },
    };
    float cc[2];

    printf("=== 用 C 的 Kin_FkNormal 核验用户轨迹（爪心位置）===\n");
    printf("  用户θ(1,2,4)          Σ_el      爪心(x,z) mm         舵机角(°)\n");
    for (int i = 0; i < 5; i++)
    {
        float el1 = -user[i][0], el2 = -user[i][1], el4 = -user[i][2];
        float servo[3] = { 90.0f - el1, 90.0f - el2, 90.0f - el4 };
        Kin_FkNormal(servo, cc);
        printf("  (%+7.2f,%+7.2f,%+7.2f)  %+7.2f   (%8.2f, %8.2f)   (%.2f, %.2f, %.2f)\n",
               user[i][0], user[i][1], user[i][2], el1+el2+el4, cc[0], cc[1],
               servo[0], servo[1], servo[2]);
    }

    printf("\n=== 用 C 的 Kin_IkNormal 反解同一批爪心点（方向取竖直向下 -90）===\n");
    printf("  目标爪心(x,z)          逆解舵机角              换算回 θ            Σ_el\n");
    for (int i = 0; i < 5; i++)
    {
        float el1 = -user[i][0], el2 = -user[i][1], el4 = -user[i][2];
        float servo[3] = { 90.0f - el1, 90.0f - el2, 90.0f - el4 };
        Kin_FkNormal(servo, cc);
        float out[3];
        bool ok = Kin_IkNormal(cc, -90.0f, out);
        if (!ok) { printf("  (%8.2f,%8.2f)   <== 失败\n", cc[0], cc[1]); continue; }
        printf("  (%8.2f,%8.2f)   (%.2f, %.2f, %.2f)   (%+7.2f,%+7.2f,%+7.2f)  %+7.2f\n",
               cc[0], cc[1], out[0], out[1], out[2],
               90.0f-out[0] ? -(90.0f-out[0]) : 0.0f,
               -(90.0f-out[1]), -(90.0f-out[2]),
               (90.0f-out[0])+(90.0f-out[1])+(90.0f-out[2]));
    }

    printf("\n=== Kin_NormalDir 一致性 ===\n");
    for (int i = 0; i < 3; i++)
    {
        float el1 = -user[i][0], el2 = -user[i][1], el4 = -user[i][2];
        float servo[3] = { 90.0f - el1, 90.0f - el2, 90.0f - el4 };
        printf("  位形 %d: Kin_NormalDir = %.3f°   (期望 -90° 附近，即竖直向下)\n",
               i, Kin_NormalDir(servo));
    }
    return 0;
}
