/**
 * @file    twin_harness.c
 * @brief   数字孪生桥 C 端（host 编译，不进固件）：吃孪生桥转来的遥控帧，
 *          跑真车同款 UserApp 业务链，吐回下行 JSON 驱动 3D 模型
 *
 * 管道协议（行式，与 PcTwin/ws_bridge.py 对应）：
 *   输入行：F,<t_ms>,<seq>,<vx>,<vy>,<omega>,<j0>..<j4>,<keys>,<flags>
 *           （十进制整数，vx/omega 可负；t_ms 由桥注入的毫秒时基）
 *           T,<mode>,<a>,<b>,<p1>,<p2> —— 坐标/姿态请求（桥转发网页面板）：
 *             mode 0 = 面内直线去 (a,b)mm，爪指向 p1°（要求爪旋转已回中）
 *             mode 1 = 沿爪开合轴进给 a mm（>0 下插 / <0 上抽），腕角 b°（0=车端默认）
 *             mode 2 = 直达预设点位 idx=a（车端 robot_config 预设表）
 *             mode 3 = 姿态宏 a：0=收拢回初始位（关节空间直达姿态表）
 *                      1=展开（爪尖竖直上抬 ROBOT_ARM_DEPLOY_RISE_MM，
 *                      爪姿态保持；折叠位出发自动降级关节空间甩到竖直终点）
 *             mode 4 = 法向平移（用户 claw_traj 语义）：a=行程 mm（正=沿当前
 *                      爪板法向推进，负=反向退出）。方向不用指定——取当前
 *                      姿态的爪板法向（Kin_NormalDir），Σ 已在当前值无需
 *                      重定向，锁 roll=0 后直线即纯平移。
 *   输出行：C <json>  下行指令（桥转发给网页，驱动 3D 模型）
 *           S <json>  本地状态（桥留档对照，不进网页）
 *
 * 坐标模式语义：直线插补在 C 链内完成（kinematics 全线预扫 + 节拍取点），
 * 目标角折算成"本拍归一化速率"混入 Grab_Update——网页和真车吃同一条速率
 * 流，见到的手感即为固件行为。急停立即取消直线；F 流断供 >200ms 同样取消
 * （防复联后直线终点跳变）。C 行 ln 字段：0 待机 / 1 插补中 / 2 请求被拒
 * （不可达/越软限位）/ 3 爪旋转自动回中中（回中到容差后自动起步）。
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
#include "kinematics.h"

/* 爪开合在孪生侧是 0..12mm 平移行程，本链用舵机 0..180° 角——桥对照时跳过
 * joint[3]，其余四关节按 (rad→deg)+90° 对表 */
#define GRIPPER_INDEX 3

/* 每关节固定起始角（= 孪生建模姿态，舵机中位 90°） */
static const float kDeg0[ROBOT_CMD_JOINT_COUNT] = { 90, 90, 90, 90, 90 };

/* ---- 坐标模式（T 行）---- */
static Kin_Line_t s_line;          /* 直线插补状态机（kinematics 持有语义） */
static int s_ln = 0;               /* C 行 ln：0 待机/1 插补中/2 拒绝/3 爪未回中 */
static uint32_t s_last_tick_ms;    /* 上一 F 拍时基（断流保护 + 速率折算 dt） */
static int s_step_fail;            /* 连续取点失败计数（超 10 拍停车） */
static int s_pending;              /* 1=TOOL 请求挂起中（自动回中后起步） */
static float s_pend_tx, s_pend_tz, s_pend_pitch;   /* 挂起请求的目标 */
/* 预设点位表（robot_config 单一来源，随 cfg 下发网页端） */
static const float kPresets[ROBOT_ARM_PRESET_COUNT][3] = ROBOT_ARM_PRESET_LIST;
/* 姿态宏参数（robot_config 单一来源）+ 状态机（与直线互斥，新请求互踢） */
static const float kStowArm[3] = ROBOT_ARM_STOW_ARM_LIST;
static Grab_PoseMove_t s_pose;
static int s_pend_kind;            /* 回中完成后的分发：1=TOOL 2=展开 3=法向平移 */
static float s_press_stroke;       /* 法向平移行程（mm，符号=沿/逆当前法向） */

/* 法向平移（用户 claw_traj 语义，2026-10-12 采纳）：方向=当前爪板法向
 * （Kin_NormalDir），Σ 已在当前值——无需重定向，一条直线即纯平移。
 * 结果写 s_ln：1 起步 2 拒绝（直线预扫拒绝：越环带/奇异/越限位） */
static void arm_press_begin(Grab_State_t *g, uint32_t now_ms)
{
    const float d3[3] = { g->deg[0], g->deg[1], g->deg[2] };
    float cc[2];
    Kin_FkNormal(d3, cc);
    float dir = Kin_NormalDir(d3);
    Kin_LineReq_t req;
    memset(&req, 0, sizeof req);
    req.mode = KIN_MODE_NORMAL;
    req.from_x_mm = cc[0];
    req.from_z_mm = cc[1];
    req.to_x_mm = cc[0] + s_press_stroke * cosf(dir * (float)M_PI / 180.0f);
    req.to_z_mm = cc[1] + s_press_stroke * sinf(dir * (float)M_PI / 180.0f);
    req.dir_deg = dir;
    s_ln = Kin_LineStart(&s_line, &req, now_ms) ? 1 : 2;
    s_last_tick_ms = now_ms;
}

/* TOOL 类请求的公共入口：从当前爪尖出发直线去 (tx,tz)，终点爪指向 pitch。
 * 起点爪指向 = 当前关节角反推（爪安装向量角 + 三关节仰角和），直线全程
 * 从当前姿态线性过渡到目标指向——避免"起点姿态下该指向不可达"的假拒绝。
 * 返回：1=已起步 / 0=几何拒绝（预扫不可达或越软限位） / 2=爪未回中，
 *       请求已挂起（自动回中流程，到位后由 F 拍起步） */
static int arm_line_goto(Grab_State_t *g, float tx, float tz, float pitch,
                         uint32_t now_ms)
{
    static const float kRad2Deg = 57.29578f;
    float roll_urdf = g->deg[4] - 90.0f;
    const float d3[3] = { g->deg[0], g->deg[1], g->deg[2] };
    Kin_Pose_t p;
    Kin_Fk(d3, 0.0f, &p);
    if (fabsf(roll_urdf) > ROBOT_ARM_ROLL_CENTER_TOL_DEG)
    {
        /* 面内模式要求爪旋转回中（几何前提）：挂起请求，自动回中后起步 */
        s_pend_tx = tx;
        s_pend_tz = tz;
        s_pend_pitch = pitch;
        s_pend_kind = 1;
        s_pending = 1;
        (void)now_ms;
        return 2;
    }
    float tool_elev = atan2f(ROBOT_ARM_TOOL_Z_MM, ROBOT_ARM_TOOL_X_MM) * kRad2Deg;
    float pitch0 = tool_elev + (90.0f - g->deg[0]) + (90.0f - g->deg[1])
                 + (90.0f - g->deg[2]);
    Kin_LineReq_t req;
    memset(&req, 0, sizeof req);
    req.mode = KIN_MODE_TOOL;
    req.from_x_mm = p.x_mm;
    req.from_z_mm = p.z_mm;
    req.to_x_mm = tx;
    req.to_z_mm = tz;
    req.tool_pitch_deg = pitch;
    req.tool_pitch0_deg = pitch0;
    return Kin_LineStart(&s_line, &req, now_ms) ? 1 : 0;
}

/* 展开：爪尖从当前位置竖直上抬 ROBOT_ARM_DEPLOY_RISE_MM，爪指向从当前值
 * 过渡到 ROBOT_ARM_DEPLOY_PITCH_DEG（保持折叠位朝下指向会让腕链倒挂、
 * 抬升够不到高度）。优先真竖直直线（TOOL 逆解，路径逐点校验）；折叠位
 * 等另一弯折分支姿态直线中段不可达 → 降级关节空间直达竖直终点（扫动
 * 不可避免，终点不变）；靠近臂长边界时自动缩步幅（满步→6 成→3 成）。
 * 结果写 s_ln：1 起步 2 拒绝 */
static void arm_deploy_start(Grab_State_t *g, uint32_t now_ms)
{
    static const float kRad2Deg = 57.29578f;
    const float d3[3] = { g->deg[0], g->deg[1], g->deg[2] };
    Kin_Pose_t p;
    Kin_Fk(d3, 0.0f, &p);
    float pitch0 = atan2f(ROBOT_ARM_TOOL_Z_MM, ROBOT_ARM_TOOL_X_MM) * kRad2Deg
                 + (90.0f - g->deg[0]) + (90.0f - g->deg[1]) + (90.0f - g->deg[2]);
    const float rise[3] = { ROBOT_ARM_DEPLOY_RISE_MM,
                            ROBOT_ARM_DEPLOY_RISE_MM * 0.6f,
                            ROBOT_ARM_DEPLOY_RISE_MM * 0.3f };
    for (int i = 0; i < 3; i++)
    {
        Kin_LineReq_t req;
        memset(&req, 0, sizeof req);
        req.mode = KIN_MODE_TOOL;
        req.from_x_mm = p.x_mm;
        req.from_z_mm = p.z_mm;
        req.to_x_mm = p.x_mm;
        req.to_z_mm = p.z_mm + rise[i];
        req.tool_pitch_deg = ROBOT_ARM_DEPLOY_PITCH_DEG;
        req.tool_pitch0_deg = pitch0;
        if (Kin_LineStart(&s_line, &req, now_ms))
        {
            s_ln = 1;
            s_last_tick_ms = now_ms;
            return;                     /* 真·竖直到位路径 */
        }
    }
    /* 直线全拒（折叠分支/贴边界）：关节空间直达竖直终点 */
    for (int i = 0; i < 3; i++)
    {
        float tip[2] = { p.x_mm, p.z_mm + rise[i] };
        float deg[3];
        if (Kin_IkTool(tip, ROBOT_ARM_DEPLOY_PITCH_DEG, deg))
        {
            float tgt[ROBOT_CMD_JOINT_COUNT] = { deg[0], deg[1], deg[2],
                                                 g->deg[3], g->deg[4] };
            Grab_PoseMoveStart(&s_pose, tgt, ROBOT_ARM_POSE_DPS, now_ms);
            s_ln = 1;
            s_last_tick_ms = now_ms;
            return;
        }
    }
    s_ln = 2;                           /* 连 3 成步幅都不可达：真贴边界 */
}

int main(void)
{
    char line[256];

    RC_Cmd_Init();
    Grab_State_t grab;
    Grab_Init(&grab, kDeg0);

    while (fgets(line, sizeof line, stdin) != NULL)
    {
        unsigned t_ms = 0u, seq = 0u, keys = 0u, flags = 0u;
        int vx = 0, vy = 0, om = 0;
        int j[ROBOT_CMD_JOINT_COUNT] = { 0 };
        bool is_frame = false;

        if (line[0] == 'T')
        {
            /* ---- 坐标模式请求（网页面板 → 桥 → 本链；桥注入 t_ms）---- */
            int m = 0;
            double a = 0.0, b = 0.0, p1 = 0.0;
            if (sscanf(line + 1, ",%u,%d,%lf,%lf,%lf",
                       &t_ms, &m, &a, &b, &p1) != 5)
            {
                continue;                       /* 残行丢弃 */
            }
            if (m == 0)
            {
                /* 面内直线去 (a,b)，爪指向 p1（要求爪旋转已回中） */
                s_pose.active = false;         /* 新请求互踢 */
                int r = arm_line_goto(&grab, (float)a, (float)b, (float)p1,
                                      (uint32_t)t_ms);
                if (r == 0)
                {
                    s_ln = 2;                      /* 几何拒绝，粘滞 */
                }
                else
                {
                    s_ln = (r == 2) ? 3 : 1;
                    s_last_tick_ms = (uint32_t)t_ms;   /* 起步/挂起都刷新断流窗 */
                }
            }
            else if (m == 1)
            {
                /* 沿开合轴进给 a mm（>0 下插/<0 上抽）：roll 用当前实际角
                 * （先转爪再进给的操作顺序），腕 b=0 取车端默认 */
                s_pose.active = false;         /* 新请求互踢 */
                float roll_urdf = grab.deg[4] - 90.0f;
                const float d3[3] = { grab.deg[0], grab.deg[1], grab.deg[2] };
                Kin_Pose_t p;
                Kin_Fk(d3, roll_urdf, &p);
                float from[2] = { p.x_mm, p.z_mm };
                float wrist = (fabs(b) < 0.5) ? ROBOT_ARM_DESCENT_WRIST_DEG
                                              : (float)b;
                Kin_LineReq_t req;
                Kin_LineDescend(&req, from, (float)a, roll_urdf, wrist);
                if (Kin_LineStart(&s_line, &req, (uint32_t)t_ms))
                {
                    s_ln = 1;
                    s_last_tick_ms = (uint32_t)t_ms;   /* 起步即刷新断流窗 */
                }
                else
                {
                    s_ln = 2;
                }
            }
            else if (m == 2)
            {
                /* 直达预设点位 idx=a（robot_config 预设表，cfg 下发网页） */
                s_pose.active = false;         /* 新请求互踢 */
                int idx = (int)a;
                int r = -2;
                if (idx >= 0 && idx < ROBOT_ARM_PRESET_COUNT)
                {
                    r = arm_line_goto(&grab, kPresets[idx][0], kPresets[idx][1],
                                      kPresets[idx][2], (uint32_t)t_ms);
                }
                if (r == 0)
                {
                    s_ln = 2;
                }
                else
                {
                    s_ln = (r == 2) ? 3 : 1;
                    s_last_tick_ms = (uint32_t)t_ms;
                }
            }
            else if (m == 4)
            {
                /* 法向平移：a=行程 mm（正=沿当前法向推进，负=反向退出） */
                s_line.active = false;
                s_pending = 0;
                s_press_stroke = (float)a;
                if (fabsf(grab.deg[4] - 90.0f) > ROBOT_ARM_ROLL_CENTER_TOL_DEG)
                {
                    s_pend_kind = 3;           /* 先自动回中再平移 */
                    s_pending = 1;
                    s_ln = 3;
                    s_last_tick_ms = (uint32_t)t_ms;
                }
                else
                {
                    arm_press_begin(&grab, (uint32_t)t_ms);
                }
            }
            else if (m == 3)
            {
                /* 姿态宏：0=收拢回初始位（关节空间直达）1=展开（竖直上抬） */
                s_line.active = false;         /* 新请求互踢 */
                s_pending = 0;
                if (a < 0.5)
                {
                    float tgt[ROBOT_CMD_JOINT_COUNT] = {
                        kStowArm[0], kStowArm[1], kStowArm[2],
                        grab.deg[3], grab.deg[4]
                    };
                    Grab_PoseMoveStart(&s_pose, tgt, ROBOT_ARM_POSE_DPS,
                                       (uint32_t)t_ms);
                    s_ln = 1;
                    s_last_tick_ms = (uint32_t)t_ms;
                }
                else if (fabsf(grab.deg[4] - 90.0f) > ROBOT_ARM_ROLL_CENTER_TOL_DEG)
                {
                    s_pend_kind = 2;           /* 展开也要求爪回中：挂起等回中 */
                    s_pending = 1;
                    s_ln = 3;
                    s_last_tick_ms = (uint32_t)t_ms;
                }
                else
                {
                    s_pose.active = false;
                    arm_deploy_start(&grab, (uint32_t)t_ms);
                }
            }
            else
            {
                s_ln = 2;
            }
        }
        else if (line[0] == 'F')
        {
            if (sscanf(line + 1, ",%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u",
                       &t_ms, &seq, &vx, &vy, &om,
                       &j[0], &j[1], &j[2], &j[3], &j[4], &keys, &flags) != 12)
            {
                continue;                       /* 残行丢弃（桥保证整行写入） */
            }
            is_frame = true;
        }
        else
        {
            continue;                           /* 空行/未知行忽略 */
        }

        if (is_frame)
        {
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
            RC_Cmd_Update(&f, (uint32_t)t_ms);
        }

        RC_Cmd_t rc;
        RC_Cmd_GetCopy(&rc);

        /* ---- 坐标模式：直线进行中用"目标角→本拍速率"顶替关节点动 ----
         * 速率 = (目标角−当前角)/(满幅°/s×dt)，一两拍内追上目标点；
         * 急停立即取消直线与回中流程（rc 快照已把速率归零，角度保持）；
         * F 流断供 >200ms（失联/页面切走）同样取消，防复联后终点跳变 */
        if (rc.estop || (uint32_t)t_ms - s_last_tick_ms > 200u)
        {
            s_line.active = false;
            s_pending = 0;
            s_pose.active = false;
        }
        float rate_used[ROBOT_CMD_JOINT_COUNT];
        for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
        {
            rate_used[i] = is_frame ? rc.joint[i] : 0.0f;   /* T 拍无新帧=角度保持 */
        }
        uint32_t dt_ms = (uint32_t)t_ms - s_last_tick_ms;
        if (dt_ms < 1u)
        {
            dt_ms = 1u;
        }
        float chase_k = 1.0f / (ROBOT_APP_JOINT_RATE_DPS * (float)dt_ms * 0.001f);
        if (s_pending)
        {
            /* 自动回中：爪旋转朝舵机 90°（URDF roll 0）追，到容差即起步 */
            float r = (90.0f - grab.deg[4]) * chase_k;
            if (r > 1.0f) r = 1.0f;
            if (r < -1.0f) r = -1.0f;
            rate_used[4] = r;
            if (fabsf(grab.deg[4] - 90.0f) <= ROBOT_ARM_ROLL_CENTER_TOL_DEG)
            {
                s_pending = 0;
                if (s_pend_kind == 2)
                {
                    arm_deploy_start(&grab, (uint32_t)t_ms);   /* 展开分发 */
                }
                else if (s_pend_kind == 3)
                {
                    arm_press_begin(&grab, (uint32_t)t_ms);    /* 法向平移分发 */
                }
                else
                {
                    int r2 = arm_line_goto(&grab, s_pend_tx, s_pend_tz,
                                           s_pend_pitch, (uint32_t)t_ms);
                    s_ln = (r2 == 1) ? 1 : ((r2 == 2) ? 3 : 2);
                    if (r2 == 1)
                    {
                        s_last_tick_ms = (uint32_t)t_ms;
                    }
                }
            }
        }
        if (s_line.active)
        {
            uint32_t elapsed = (uint32_t)t_ms - s_line.t0_ms;
            float tgt[3];
            if (elapsed == 0u)
            {
                s_step_fail = 0;        /* 起点拍：目标=当前位姿，保持即可 */
            }
            else if (Kin_LineStep(&s_line, (uint32_t)t_ms, tgt))
            {
                s_step_fail = 0;
                for (int i = 0; i < 3; i++)
                {
                    float r = (tgt[i] - grab.deg[i]) * chase_k;
                    if (r > 1.0f) r = 1.0f;
                    if (r < -1.0f) r = -1.0f;
                    rate_used[i] = r;
                }
            }
            else if (++s_step_fail > 10)
            {
                s_line.active = false;  /* 连续取点失败=数值异常，停车 */
            }
            /* 容忍窗内的取点失败（起点奇异邻域）→ 本拍保持当前角等进度走出 */
        }
        else if (s_pose.active)
        {
            /* 姿态宏：Step 出速率喂 Grab_Update（deg 唯一写者不变） */
            float prate[ROBOT_CMD_JOINT_COUNT];
            if (Grab_PoseMoveStep(&s_pose, &grab, (uint32_t)t_ms, prate))
            {
                for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
                {
                    rate_used[i] = prate[i];
                }
            }
        }
        s_ln = (s_line.active || s_pose.active)
             ? 1 : ((s_ln == 2 || s_ln == 3) ? s_ln : 0);
        s_last_tick_ms = (uint32_t)t_ms;

        Chassis_Vec_t raw, shaped, phys;
        raw.vx = rc.vx; raw.vy = rc.vy; raw.omega = rc.omega;
        float rpm[CHASSIS_WHEEL_COUNT];
        Chassis_Shape(&raw, &shaped);
        Chassis_ToPhysical(&shaped, &phys);
        Chassis_MecanumIK(&phys, rpm);

        Grab_Update(&grab, rate_used, (uint32_t)t_ms);

        /* ---- 爪尖遥测（kinematics FK）：面内坐标 + 横向偏移 + 开合方向 ----
         * roll 舵机角 90° = URDF roll 0（对表口径同 (rad→deg)+90°） */
        const float arm_deg[3] = { grab.deg[0], grab.deg[1], grab.deg[2] };
        Kin_Pose_t tip;
        Kin_Fk(arm_deg, grab.deg[4] - 90.0f, &tip);

        /* ---- 下行 JSON：整形底盘逻辑量 + 合成关节速率 + 急停闩锁 + 直线状态 ---- */
        printf("C {\"type\":\"command\",\"seq\":%u,"
               "\"chassis\":{\"vx\":%ld,\"vy\":%ld,\"omega\":%ld},"
               "\"joint\":[%ld,%ld,%ld,%ld,%ld],\"flags\":%u,\"ln\":%d}\n",
               (unsigned)rc.seq,
               lround((double)shaped.vx * 1000.0),
               lround((double)shaped.vy * 1000.0),
               lround((double)shaped.omega * 1000.0),
               lround((double)grab.rate[0] * 100.0),
               lround((double)grab.rate[1] * 100.0),
               lround((double)grab.rate[2] * 100.0),
               lround((double)grab.rate[3] * 100.0),
               lround((double)grab.rate[4] * 100.0),
               rc.estop ? 1u : 0u,
               s_ln);

        /* ---- 本地状态：目标角 + 四轮目标转速 + 满量程/预设配置（桥转发网页）----
         * cfg = robot_config.h 的 ROBOT_APP_* 宏直出：满量程真值以编译进本链的
         * 这份常量为准，桥与网页端都不再各自维护第二份；预设点位同源下发。 */
        printf("S {\"t\":%u,\"deg\":[%.2f,%.2f,%.2f,%.2f,%.2f],"
               "\"rpm\":[%.2f,%.2f,%.2f,%.2f],"
               "\"tip\":[%.1f,%.1f],\"tipy\":%.1f,"
               "\"open\":[%.3f,%.3f],\"ln\":%d,"
               "\"cfg\":{\"vx_max_mps\":%.3f,\"vy_max_mps\":%.3f,"
               "\"w_max_radps\":%.3f,\"joint_rate_dps\":%.1f,\"presets\":[",
               (unsigned)t_ms,
               grab.deg[0], grab.deg[1], grab.deg[2],
               grab.deg[3], grab.deg[4],
               rpm[0], rpm[1], rpm[2], rpm[3],
               (double)tip.x_mm, (double)tip.z_mm, (double)tip.y_mm,
               (double)tip.open_x, (double)tip.open_z,
               s_ln,
               (double)ROBOT_APP_VX_MAX_MPS, (double)ROBOT_APP_VY_MAX_MPS,
               (double)ROBOT_APP_W_MAX_RADPS, (double)ROBOT_APP_JOINT_RATE_DPS);
        for (int i = 0; i < ROBOT_ARM_PRESET_COUNT; i++)
        {
            printf("%s[%.1f,%.1f,%.1f]", i ? "," : "", (double)kPresets[i][0],
                   (double)kPresets[i][1], (double)kPresets[i][2]);
        }
        printf("]}}\n");
        fflush(stdout);
    }
    return 0;
}
