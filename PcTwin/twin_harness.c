/**
 * @file    twin_harness.c
 * @brief   数字孪生桥 C 端（host 编译，不进固件）：吃孪生桥转来的遥控帧，
 *          跑真车同款 UserApp 业务链，吐回下行 JSON 驱动 3D 模型
 *
 * 管道协议（行式，与 PcTwin/ws_bridge.py 对应）：
 *   输入行：F,<t_ms>,<seq>,<vx>,<vy>,<omega>,<j0>..<j4>,<keys>,<flags>
 *           （十进制整数，vx/omega 可负；t_ms 由桥注入的毫秒时基）
 *   输出行：C <json>  下行指令（桥转发给网页，驱动 3D 模型）
 *           S <json>  本地状态（桥留档对照，不进网页）
 *
 * 关键设计：这里跑的就是固件里那几份源文件（rc_cmd/chassis_ctrl/grab_ctrl
 * 原样编译），不是复刻逻辑——孪生上看到的手感/急停/渐停即为真车行为；
 * FreeRTOS 快照锁由 freertos_stub 空实现顶替（同 PcTests 先例）。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "rc_cmd.h"
#include "chassis_ctrl.h"
#include "grab_ctrl.h"

/* 爪开合在孪生侧是 0..12mm 平移行程，本链用舵机 0..180° 角——桥对照时跳过
 * joint[3]，其余四关节按 (rad→deg)+90° 对表 */
#define GRIPPER_INDEX 3

/* 每关节固定起始角（= 孪生建模姿态，舵机中位 90°） */
static const float kDeg0[ROBOT_CMD_JOINT_COUNT] = { 90, 90, 90, 90, 90 };

int main(void)
{
    char line[256];

    RC_Cmd_Init();
    Grab_State_t grab;
    Grab_Init(&grab, kDeg0);

    while (fgets(line, sizeof line, stdin) != NULL)
    {
        unsigned t_ms, seq, keys, flags;
        int vx, vy, om, j[ROBOT_CMD_JOINT_COUNT];

        if (line[0] != 'F')
        {
            continue;                       /* 空行/未知行忽略 */
        }
        if (sscanf(line + 1, ",%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u",
                   &t_ms, &seq, &vx, &vy, &om,
                   &j[0], &j[1], &j[2], &j[3], &j[4], &keys, &flags) != 12)
        {
            continue;                       /* 残行丢弃（桥保证整行写入） */
        }

        /* ---- 15B 帧装配（整帧清零再填，同发送端纪律）---- */
        RCPayload_t f;
        memset(&f, 0, sizeof f);
        f.seq = (uint8_t)seq;
        f.vx = (int16_t)vx;
        f.vy = (int16_t)vy;
        f.omega = (int16_t)om;
        for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
        {
            f.joint[i] = (int8_t)j[i];
        }
        f.keys = (uint16_t)keys;
        f.flags = (uint8_t)flags;

        /* ---- 真车同款业务链：解析 → 手感 → 量纲 → 逆解 → 关节积分 ---- */
        RC_Cmd_Update(&f, (uint32_t)t_ms);

        RC_Cmd_t rc;
        RC_Cmd_GetCopy(&rc);

        Chassis_Vec_t raw, shaped, phys;
        raw.vx = rc.vx; raw.vy = rc.vy; raw.omega = rc.omega;
        float rpm[CHASSIS_WHEEL_COUNT];
        Chassis_Shape(&raw, &shaped);
        Chassis_ToPhysical(&shaped, &phys);
        Chassis_MecanumIK(&phys, rpm);

        Grab_Update(&grab, rc.joint, (uint32_t)t_ms);

        /* ---- 下行 JSON：整形后的底盘逻辑量 + 合成关节速率 + 急停闩锁态 ---- */
        printf("C {\"type\":\"command\",\"seq\":%u,"
               "\"chassis\":{\"vx\":%ld,\"vy\":%ld,\"omega\":%ld},"
               "\"joint\":[%ld,%ld,%ld,%ld,%ld],\"flags\":%u}\n",
               (unsigned)rc.seq,
               lround((double)shaped.vx * 1000.0),
               lround((double)shaped.vy * 1000.0),
               lround((double)shaped.omega * 1000.0),
               lround((double)grab.rate[0] * 100.0),
               lround((double)grab.rate[1] * 100.0),
               lround((double)grab.rate[2] * 100.0),
               lround((double)grab.rate[3] * 100.0),
               lround((double)grab.rate[4] * 100.0),
               rc.estop ? 1u : 0u);

        /* ---- 本地状态：目标角 + 四轮目标转速 + 满量程配置（桥转发网页）----
         * cfg = robot_config.h 的 ROBOT_APP_* 宏直出：满量程真值以编译进本链的
         * 这份常量为准，桥与网页端都不再各自维护第二份。 */
        printf("S {\"t\":%u,\"deg\":[%.2f,%.2f,%.2f,%.2f,%.2f],"
               "\"rpm\":[%.2f,%.2f,%.2f,%.2f],"
               "\"cfg\":{\"vx_max_mps\":%.3f,\"vy_max_mps\":%.3f,"
               "\"w_max_radps\":%.3f,\"joint_rate_dps\":%.1f}}\n",
               (unsigned)t_ms,
               grab.deg[0], grab.deg[1], grab.deg[2],
               grab.deg[3], grab.deg[4],
               rpm[0], rpm[1], rpm[2], rpm[3],
               (double)ROBOT_APP_VX_MAX_MPS, (double)ROBOT_APP_VY_MAX_MPS,
               (double)ROBOT_APP_W_MAX_RADPS, (double)ROBOT_APP_JOINT_RATE_DPS);
        fflush(stdout);
    }
    return 0;
}
