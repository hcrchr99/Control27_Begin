/**
 * @file    kinematics.h
 * @brief   机械臂平面运动学纯函数（Modules 规划 §3.9）：爪尖坐标 ↔ 三关节角，
 *          两种直线运动模式 + 全线预扫，零硬件依赖
 *
 * ── 管什么 ─────────────────────────────────────────────────────────
 * 只算"爪尖要在臂面内某个点，三个关节该转到多少度"。大臂/小臂/腕俯仰
 * （joint[0..2]）归本模块；爪开合（joint[3]）与爪旋转（joint[4]）是工具
 * 自由度，由调用方直接给角度，不经过这里。唯一消费者是 grab_task。
 *
 * ── 两种运动模式 ───────────────────────────────────────────────────
 *  KIN_MODE_TOOL  面内坐标模式（爪旋转保持 0）：给定爪尖目标点 + 爪安装
 *                 向量的绝对指向，逆解三关节。水平进给/竖直下压、预设
 *                 点位、爪尖任意直线都走这个模式。
 *  KIN_MODE_OPEN  沿开合方向模式：爪旋转转到 ROBOT_ARM_DESCENT_ROLL_DEG
 *                 后，爪子两指的开合方向从水平横向摆到接近竖直（还差
 *                 29.3°，这是斜置转轴的几何极限）。此模式下爪尖沿开合
 *                 方向进给（下插抓矿/上抽退出），腕角全程恒定。
 *
 * ── 为什么两种模式要分开算 ────────────────────────────────────────
 * 腕旋转轴是斜置的（臂面内、与水平约 29.3°，见 ROBOT_ARM_ROLL_AX_*），
 * 且轴心不在臂面上：爪旋转一转，腕俯仰轴和爪尖都会被甩出臂面（roll
 * 从 0° 到 90°，夹取点横移约 56→124mm，面内位置也变约 76mm）。所以
 * roll≠0 时不能套用 roll=0 的平面逆解——本模块把"旋转后的腕+爪"折算
 * 成一根随 roll 变化的等效连杆，仍用闭式逆解，不迭代。爪开合方向本身
 * 只随 roll 变（其余关节轴都平行于开合方向，转它们不改开合方向），
 * 这一点让 OPEN 模式的直线方向可以闭式算出。
 *
 * ── 坐标与角度口径 ────────────────────────────────────────────────
 *  长度 mm，坐标原点 = base_link 原点（地面投影中心），x 前 / z 上；
 *  关节角 = 舵机 0..180° 口径，建模姿态（links.json 全零角）= 中位 90°；
 *  爪旋转角（roll）用 URDF 口径 0..180°（links.json 的 wrist_roll_joint
 *  转角），与舵机角差一个安装标定，实机对表后在调用方换算。
 *  几何常量全部来自 robot_config.h（CAD 同源，实机标定回写只改那里）。
 */
#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

/* FK 输出：爪尖位置与爪开合方向 */
typedef struct
{
    float x_mm, z_mm;   /* 夹取中点在臂面内的位置 */
    float y_mm;         /* 夹取点横向偏移（出臂面多少，底盘对准要补偿的量） */
    float open_x, open_z; /* 开合方向在臂面内的投影（单位向量；roll=0 时
                           * 开合纯横向，投影为零向量——不是没方向） */
} Kin_Pose_t;

/* 直线请求（Kin_LineStart 用，调用方栈上现配现用） */
typedef struct
{
    int mode;               /* KIN_MODE_TOOL / KIN_MODE_OPEN / KIN_MODE_NORMAL */
    float from_x_mm, from_z_mm; /* 起点 = 当前爪尖（调用方用 Kin_Fk 算出来填） */
    float to_x_mm, to_z_mm;     /* 终点 */
    float tool_pitch_deg;   /* TOOL 模式：终点爪安装向量绝对指向 */
    float tool_pitch0_deg;  /* TOOL 模式：起点爪指向（与终点不同则全程线性
                             * 过渡——从任意当前姿态出发时爪边走边转，避免
                             * "起点姿态下该指向不可达"的假拒绝；两点相同
                             * 即全程恒定） */
    float roll_deg;         /* OPEN 模式：爪旋转角（全程恒定） */
    float wrist_deg;        /* OPEN 模式：腕舵机角（全程恒定） */
    float dir_deg;          /* NORMAL 模式：法向运动方向（面内仰角，度；
                             * −90=竖直向下压，0=水平前推；Σ 随之定死） */
} Kin_LineReq_t;

/* 直线插补状态机（grab_task 持有静态一份，同 §3.9 KinLine_t） */
typedef struct
{
    bool active;            /* true = 插补中；Step 到终点后自动清 false */
    int mode;
    float from_x_mm, from_z_mm, to_x_mm, to_z_mm;
    float tool_pitch_deg, tool_pitch0_deg, roll_deg, wrist_deg, dir_deg;
    uint32_t t0_ms;         /* 起步时基（Step 用 now-t0 算进度） */
    uint32_t total_ms;      /* 全程时长 = 长度/线速度 */
} Kin_Line_t;

#define KIN_MODE_TOOL 0
#define KIN_MODE_OPEN 1
#define KIN_MODE_NORMAL 2   /* 爪板沿自身法向纯平移（Σ=θ1+θ2+θ4 恒定，roll 锁 0）*/

/* ---- 正逆解 ---- */

/** 正解：三关节舵机角 + 爪旋转角 → 爪尖位置与开合方向（含横向偏移） */
void Kin_Fk(const float deg[3], float roll_deg, Kin_Pose_t *out);

/** TOOL 逆解：爪尖目标点 + 爪安装向量绝对指向 → 三关节舵机角
 *  @return false = 不可达/伸直奇异/越软限位（out 不动） */
bool Kin_IkTool(const float tip_mm[2], float tool_pitch_deg, float deg_out[3]);

/** OPEN 逆解：爪尖目标点 + 爪旋转角 + 恒定腕角 → 三关节舵机角
 *  （腕角原样进原样出，凑齐 deg[3] 便于调用方统一喂执行器） */
bool Kin_IkOpen(const float tip_mm[2], float roll_deg, float wrist_deg,
                float deg_out[3]);

/** NORMAL 逆解（爪板沿法向纯平移，2026-10-12 采纳用户 claw_traj 标定）：
 *  三俯仰关节轴均∥y，锁 roll=0 后爪板朝向只由 Σ=θ1+θ2+θ4 决定。
 *  给定爪心目标点与运动方向（=法向），Σ 定死，腕心偏移成常向量，
 *  剩余为闭式 2R（θ4=Σ−θ1−θ2 收尾）。
 *  @param cc_mm    爪心（TCP）目标点，臂面内 mm
 *  @param dir_deg  运动方向仰角：−90=竖直向下压，0=水平前推
 *  @param deg_out  出三关节舵机角（roll 不在此列，调用方锁 90=URDF 0） */
bool Kin_IkNormal(const float cc_mm[2], float dir_deg, float deg_out[3]);

/** NORMAL 正解：三关节舵机角（roll 任意，不参与）→ 爪心位置（臂面内）。
 *  Σ = 三关节仰角代数和，爪心 = 腕俯仰轴位 + Rot(Σ)·R4 */
void Kin_FkNormal(const float deg[3], float cc_out[2]);

/** 当前姿态的爪板法向仰角（度）：法向平移的运动方向=它，无需外部指定。
 *  与 Kin_FkNormal 同一套 Σ（方向=爪板面朝的方向，正行程=沿法向推进） */
float Kin_NormalDir(const float deg[3]);

/** 爪开合方向在臂面内的投影（单位长度=sin(roll)，roll=0 时全横向投影为零） */
void Kin_OpeningDir(float roll_deg, float out[2]);

/* ---- 直线插补（§3.9：插补节拍是时间主人）---- */

/** 起步：全线预扫（每步逆解 + 关节峰值速率校验），任一点不可达/越限/
 *  超速即拒绝并返回 false（状态机保持 inactive，轨迹一步都不动） */
bool Kin_LineStart(Kin_Line_t *l, const Kin_LineReq_t *req, uint32_t now_ms);

/** 便捷构造：OPEN 模式"从 from 沿开合方向进给"的直线请求
 *  （drop>0 = 爪沿开合轴下插；drop<0 = 反向上抽退出） */
void Kin_LineDescend(Kin_LineReq_t *req, const float from_mm[2],
                     float drop_mm, float roll_deg, float wrist_deg);

/** 节拍推进：按已流逝时间在直线上取点逆解，输出三关节舵机角。
 *  @return true = 本拍有新目标（*l 到终点时输出终点角并自动转 inactive）；
 *          false = 本拍取点失败（起点奇异邻域/数值异常）——*l 保持 active，
 *          调用方拍板：容错窗内保持当前角等进度走出，或连续失败后取消 */
bool Kin_LineStep(Kin_Line_t *l, uint32_t now_ms, float deg_out[3]);

#endif /* KINEMATICS_H */
