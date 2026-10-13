/* 找出让 Kin_Fk 与数字孪生实测 TCP0/NORMAL0 一致的几何修正
 * 孪生实测：TCP0 = (312.750, 110.020)   法向 n0 = (0.6356, 0.7720) → 仰角 50.535°
 * 现 C 的 FK(90,90,90) = (319.800, 104.198)，法向仰角 50.535°（法向是对的）
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
    float ux = ROBOT_ARM_UPPER_X_MM, uz = ROBOT_ARM_UPPER_Z_MM;
    float rx = ROBOT_ARM_ROLLJ_X_MM, rz = ROBOT_ARM_ROLLJ_Z_MM;
    float wx = ROBOT_ARM_WRIST_X_MM, wz = ROBOT_ARM_WRIST_Z_MM;
    float gx = ROBOT_ARM_TOOL_X_MM, gz = ROBOT_ARM_TOOL_Z_MM;

    float sx = ROBOT_ARM_SHOULDER_X_MM, sz = ROBOT_ARM_SHOULDER_Z_MM;
    float rollj_x = sx + ux + rx, rollj_z = sz + uz + rz;   /* 腕转关节轴心（roll centre） */

    printf("=== 现状分解（建模姿态，所有关节角=0）===\n");
    printf("  肩           (%.3f, %.3f)\n", sx, sz);
    printf("  腕转轴心 R0  (%.3f, %.3f)   [肩+U+ROLLJ]\n", rollj_x, rollj_z);
    printf("  ROLLJ→WRIST 局部 (%.3f, %.3f)  |·|=%.3f 仰角 %.4f°\n",
           wx, wz, sqrtf(wx*wx+wz*wz), atan2f(wz, wx)*R2D);
    printf("  WRIST→TOOL  局部 (%.3f, %.3f)  |·|=%.3f 仰角 %.4f°\n",
           gx, gz, sqrtf(gx*gx+gz*gz), atan2f(gz, gx)*R2D);

    /* 现状 FK(90,90,90) */
    float servo[3] = {90,90,90};
    Kin_Pose_t p; Kin_Fk(servo, 0.0f, &p);
    printf("\n  现状 FK(90,90,90) = (%.4f, %.4f)   实测 TCP0 = (%.3f, %.3f)  差 %.4f mm\n",
           p.x_mm, p.z_mm, ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM,
           sqrtf(powf(p.x_mm-ROBOT_ARM_TCP_X_MM,2)+powf(p.z_mm-ROBOT_ARM_TCP_Z_MM,2)));

    /* 反推：腕俯仰轴位置 = 肩 + U + (ROLLJ→WRIST)  (roll=0, Ry(90°) 不改和向量方向)
     * 实测法向仰角 = 50.535° → 爪安装向量绝对仰角 = 50.535 - 90 = -39.465° */
    float fore_x = rx + wx, fore_z = rz + wz;
    float wp_x = sx + ux + fore_x, wp_z = sz + uz + fore_z;
    printf("\n  腕俯仰轴位 = (%.3f, %.3f)\n", wp_x, wp_z);

    float need_x = ROBOT_ARM_TCP_X_MM - wp_x;
    float need_z = ROBOT_ARM_TCP_Z_MM - wp_z;
    printf("  实测要求 WRIST→TOOL 的合成向量（局部） = (%.3f, %.3f)  |·|=%.3f 仰角 %.4f°\n",
           need_x, need_z, sqrtf(need_x*need_x+need_z*need_z),
           atan2f(need_z, need_x)*R2D);

    /* eff_link 口径：局部工具向量 g 被 Ry(90+el3)=Ry(90) 旋转成 (gz, -gx) */
    /* 要求 (gz, -gx) == (need_x, need_z)  =>  gz = need_x, gx = -need_z */
    float gx_new = -need_z, gz_new = need_x;
    printf("\n  ⇒ 修正后的 TOOL_X_MM = %.4f   TOOL_Z_MM = %.4f\n", gx_new, gz_new);
    printf("     （现在分别是 %.4f / %.4f）\n", gx, gz);
    printf("     |G| 现在 %.4f → 修正后 %.4f\n",
           sqrtf(gx*gx+gz*gz), sqrtf(gx_new*gx_new+gz_new*gz_new));

    /* 用修正值重算 FK */
    float t_ux=ux, t_uz=uz, t_rx=rx, t_rz=rz, t_wx=wx, t_wz=wz, t_gx=gx_new, t_gz=gz_new;
    /* 直接手算 eff_link(roll=0, el3=0) */
    float a3 = 0.0f;
    float ca3 = cosf(a3), sa3 = sinf(a3);
    float wxx = t_wx + t_gx*ca3 - t_gz*sa3;
    float wzz = t_wz + t_gx*sa3 + t_gz*ca3;
    float wy  = ROBOT_ARM_WRIST_Y_MM + ROBOT_ARM_TOOL_Y_MM;  /* roll=0 → 回臂面 */
    float ax = ROBOT_ARM_ROLL_AX_X, az = ROBOT_ARM_ROLL_AX_Z;
    float px = -az, pz = ax;
    float adotw = ax*wxx + az*wzz;
    float cr = 1.0f, sr = 0.0f;
    float tx = cr*wxx + sr*wy*px + (1.0f-cr)*adotw*ax;
    float tz = cr*wzz + sr*wy*pz + (1.0f-cr)*adotw*az;
    float bx = t_rx + tx, bz = t_rz + tz;
    float len_b = sqrtf(bx*bx+bz*bz), base_b = atan2f(bz,bx);
    float beta1 = atan2f(t_uz, t_ux);
    float la = sqrtf(t_ux*t_ux+t_uz*t_uz);
    float x = sx + la*cosf(beta1) + len_b*cosf(base_b);
    float z = sz + la*sinf(beta1) + len_b*sinf(base_b);
    printf("\n  用修正 TOOL 重算建模位爪心 = (%.4f, %.4f)   目标 (%.3f, %.3f)\n",
           x, z, ROBOT_ARM_TCP_X_MM, ROBOT_ARM_TCP_Z_MM);
    printf("  残差 = %.6f mm\n",
           sqrtf(powf(x-ROBOT_ARM_TCP_X_MM,2)+powf(z-ROBOT_ARM_TCP_Z_MM,2)));
    return 0;
}
