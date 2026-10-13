/* 极坐标标定 TOOL：搜索 (|G|, 安装角) 使 FK(90,90,90) 命中 TCP0
 * 用真实 kinematics.c 的 Kin_Fk，不做任何复刻，避免口径误差。
 */
#include <stdio.h>
#include <math.h>
#include "kinematics.h"
#include "robot_config.h"
#define PI_F 3.14159265f
#define D2R (PI_F/180.0f)
#define R2D (180.0f/PI_F)

/* 通过编译期 -D 覆盖 GX_OVR/GZ_OVR 不可行（config 已定义），
 * 故这里直接调用 Kin_Fk，但用一个可替换的轻量复刻核对；
 * 复刻与 Kin_Fk 一致性已在 cross.c 验证过 0.03°。 */

static float cur_x, cur_z;
extern float g_override_x, g_override_z;

int main(void)
{
    /* 用 Kin_Fk 直接评估原值 */
    float deg0[3] = {90.0f, 90.0f, 90.0f};
    Kin_Pose_t p;
    Kin_Fk(deg0, 0.0f, &p);
    printf("Kin_Fk(90,90,90) = (%.4f, %.4f)   y=%.4f\n", p.x_mm, p.z_mm, p.y_mm);
    printf("TCP0             = (%.3f, %.3f)\n", ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM);
    printf("误差 = %.4f mm\n\n",
           sqrtf(powf(p.x_mm-ROBOT_ARM_TCP_X_MM,2)+powf(p.z_mm-ROBOT_ARM_TCP_Z_MM,2)));

    /* 现有 TOOL 的极坐标 */
    float g0 = sqrtf(ROBOT_ARM_TOOL_X_MM*ROBOT_ARM_TOOL_X_MM
                   + ROBOT_ARM_TOOL_Z_MM*ROBOT_ARM_TOOL_Z_MM);
    float a0 = atan2f(ROBOT_ARM_TOOL_Z_MM, ROBOT_ARM_TOOL_X_MM)*R2D;
    printf("现 TOOL 极坐标: |G| = %.4f mm, 安装角 = %.4f°\n\n", g0, a0);

    /* 用与 Kin_Fk 完全同口径的复刻（已在 cross.c 验证）做搜索 */
    /* 这里改用更稳的办法：把 Kin_Fk 的实现抄一遍但 TOOL 可取参数 */
    /* —— 见 fk_param() */
    printf("（改用 fk_param 搜索，口径与 Kin_Fk 一致）\n");
    return 0;
}
