/**
 * @file    rc_input.h
 * @brief   发送端输入适配：船型开关 + 摇杆原始读数 → RC_Cmd v2 协议帧（纯函数）
 *
 * 职责边界（输入适配归发送端，2026-10-11 定案）：发送端只把物理输入整理成
 * 逻辑量（设备清噪死区、按 键名→帧字段 映射填帧）；满量程/手感/物理换算
 * 归车端，本模块不做。整帧清零再填（发送端纪律）也在此收口。
 *
 * 帧字段语义（rc_cmd.h v2 冻结布局）：
 *   vx/vy      = 轴逻辑量 ±ROBOT_CMD_RANGE（采集侧已适配产出：PHYS=摇杆原始值经
 *                RcInput_NormalizeAxis 去死区归一化；WEB=网页直传逻辑量，Build 只钳位）
 *   omega      = K_6 方向刀：A → +key_omega_rate，B → −key_omega_rate（整车旋转，
 *                2026-10-16 拍板）；中位（全断）→ 0；A/B 同 1 矛盾 → 0（失安全）
 *   joint[k]   = K_(k+1) 方向刀：A 刀 → +key_joint_rate，B 刀 → −key_joint_rate，
 *                中位（全断）→ 0；同开关 A/B 同 1 = 矛盾输入 → 0（失安全）
 *   keys       = 方向刀位图透传（位布局见下方宏组；含义归 UserApp）
 *   flags      = bit0 estop（K_EB 按下置 1；车端闩锁、松键帧解除）
 *   seq        = 调用方递增透传（只在本帧实发时 ++，与 PTX 纪律一致）
 *
 * 纯函数无硬件依赖：只 include rc_cmd.h，配置经参数注入（PcTests 友好；
 * 固件侧默认值由 remoter_app.c 用 robot_config.h 的宏组装）。
 */
#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdint.h>
#include "rc_cmd.h"

/* 输入适配配置（遥控器标定参数集中注入） */
typedef struct
{
    uint16_t joy_center;    /* PHYS 摇杆中点（raw counts，实测标定）；WEB 源不消费 */
    uint16_t joy_deadband;  /* PHYS 设备清噪死区（counts）；WEB 源不消费 */
    int16_t  key_joint_rate;/* K_1..K_5 按住的关节速率（±ROBOT_CMD_JOINT_RANGE 内） */
    int16_t  key_omega_rate;/* K_6 按住的整车旋转速率（±ROBOT_CMD_RANGE 内） */
} RcInput_Config_t;

/* 一拍输入快照（采集侧填，RcInput_Build 消费）。轴已是逻辑量：
 * PHYS 采集侧调 RcInput_NormalizeAxis 产出；WEB 采集侧取串口帧直传 */
typedef struct
{
    int16_t  vx;            /* 轴逻辑量 ±ROBOT_CMD_RANGE（Build 只做钳位） */
    int16_t  vy;
    uint16_t keys;          /* 方向刀位图（位布局见下方宏组），1=该刀接地 */
    uint8_t  estop;         /* 1 = K_EB 按下（急停） */
} RcInput_Raw_t;

/* ---- 方向刀位图布局（uint16，自复位船型开关：拨上=A 通/拨下=B 通/松手回中全断）----
 * 每只开关占相邻两位：bit[2n] = 第 n 只 A 刀（正向），bit[2n+1] = B 刀（反向）：
 *   bit0/1   = K_1 A/B（大臂）  → joint[0] +/-
 *   bit2/3   = K_2 A/B（小臂）  → joint[1] +/-
 *   bit4/5   = K_3 A/B（爪子）  → joint[2] +/-（物理分组与舵机 ID 的错位待上手核对）
 *   bit6/7   = K_4 A/B          → joint[3] +/-
 *   bit8/9   = K_5 A/B（手腕）  → joint[4] +/-
 *   bit10/11 = K_6 A/B          → 整车旋转 omega +/-（2026-10-16 拍板）
 *   bit12    = K_EB 急停（消抖字内携带，拆帧入 estop 字段）
 *   bit13    = K_SW 摇杆按压（预留）
 *   bit14/15 = 预留（可放将来的双击派生位）
 * ⚠ 电气假设（待上手实测）：A/B 互斥、中位双断。同开关 A/B 同 1 = 矛盾
 *   输入，Build 按失安全处理为 0。若实物是"双刀同动作"型（A/B 恒
 *   同态），12 独立位不成立——届时改 Build 的矛盾判定一行即可。
 * 双击识别的候选刀位 = K_1..K_5 各刀（K_6 已承担整车旋转）。 */
#define RCINPUT_KEY_K1_A        (1u << 0)
#define RCINPUT_KEY_K1_B        (1u << 1)
#define RCINPUT_KEY_K2_A        (1u << 2)
#define RCINPUT_KEY_K2_B        (1u << 3)
#define RCINPUT_KEY_K3_A        (1u << 4)
#define RCINPUT_KEY_K3_B        (1u << 5)
#define RCINPUT_KEY_K4_A        (1u << 6)
#define RCINPUT_KEY_K4_B        (1u << 7)
#define RCINPUT_KEY_K5_A        (1u << 8)
#define RCINPUT_KEY_K5_B        (1u << 9)
#define RCINPUT_KEY_K6_A        (1u << 10)
#define RCINPUT_KEY_K6_B        (1u << 11)
#define RCINPUT_KEY_ESTOP_BIT   (1u << 12)
#define RCINPUT_KEY_SW_BIT      (1u << 13)
/* 预留位掩码：bit14/15（双击等派生语义未定案，Build 不消费） */
#define RCINPUT_KEY_DIR_MASK    0x0FFFu

/**
 * @brief  PHYS 摇杆单轴适配：原始 ADC → 轴逻辑量（去死区→线性拉满→钳位 ±1000）
 * @param  raw       摇杆原始值 0..4095
 * @param  center    松手中点（raw counts，实测标定）
 * @param  deadband  设备清噪死区（counts）：|raw-center|≤死区输出 0
 * @retval 轴逻辑量 ±ROBOT_CMD_RANGE
 * @note   仅物理电位器路径使用（WEB 源发送的已是逻辑量，直接进 Build）。
 *         正/负半程分开算跨度（中点一般不在正中，两端物理行程不等长）；
 *         整数运算，结果可精确复现（PcTests 断言用同式手算值）。
 */
int16_t RcInput_NormalizeAxis(uint16_t raw, uint16_t center, uint16_t deadband);

/**
 * @brief  两拍一致消抖（纯函数，采集侧每拍调用）
 * @param  debounced 上拍输出态
 * @param  prev_raw  上拍原始读数
 * @param  raw_now   本拍原始读数
 * @retval 新消抖态。某位仅当"本拍与上拍原始读数相同"时才翻转到该值，
 *         否则维持原态——孤立的单拍毛刺（机械弹跳）被整拍滤除。
 * @note   翻转固有延迟 1~2 个采样拍（10ms 拍 = 10~20ms），对自复位开关的
 *         点动语义无感；急停啮合晚至多一拍，在 100Hz 帧距下可忽略。位间独立。
 */
uint16_t RcInput_Debounce(uint16_t debounced, uint16_t prev_raw, uint16_t raw_now);

/* ---- 双击识别（事件）与 flags 编码 ----
 * flags 字节协议扩展（两端契约；bit0=estop 为 rc_cmd v2 冻结项不变）：
 *   bit1..bit6 = DCLICK1..6 双击事件位——哪路信号发生双击，对应位置 1。
 *     发送端检测到双击后保持 ROBOT_DCLICK_HOLD_MS（抗单帧丢失）；
 *     车端按 0→1 上升沿切换模式（不是电平语义），用完即弃。
 *   bit7 = 预留。
 * 双击可能出现在不止一个按键上（如 K_1A 双击=快/慢、K_5A 双击=关节/坐标），
 * 所以按"每路一位"编码而不是单一事件位。 */
#define RCINPUT_FLAG_DCLICK1    0x02u
#define RCINPUT_FLAG_DCLICK2    0x04u
#define RCINPUT_FLAG_DCLICK3    0x08u
#define RCINPUT_FLAG_DCLICK4    0x10u
#define RCINPUT_FLAG_DCLICK5    0x20u
#define RCINPUT_FLAG_DCLICK6    0x40u
#define RCINPUT_FLAG_SPARE      0x80u

/* 双击参数（robot_config.h 给默认值） */
typedef struct
{
    uint16_t window_ms;     /* 双击窗：两次按下沿的最大间隔 */
    uint8_t  hold_ticks;    /* 事件保持拍数（=HOLD_MS/发送周期，抗丢帧） */
} RcInput_DclickCfg_t;

/* 单路双击检测器状态（调用方持有，每路独立；上电清零即初始化） */
typedef struct
{
    uint8_t  prev_level;      /* 上拍电平（按下沿检测） */
    uint8_t  armed;           /* 1 = 已有第一按，等窗内第二按 */
    uint32_t first_press_ms;  /* 第一按时刻（调用方时基） */
    uint8_t  hold_left;       /* 事件保持剩余拍数 */
} RcInput_DclickState_t;

/**
 * @brief  清零双击检测器（上电/失联恢复时调用）
 */
void RcInput_DclickReset(RcInput_DclickState_t *st);

/**
 * @brief  双击检测步进（每拍对每路使能信号调用一次，纯函数）
 * @param  st      该路检测器状态
 * @param  level   本拍消抖后电平（1=按下）
 * @param  now_ms  当前时刻（调用方时基，uint32 回绕安全：只用无符号差分）
 * @param  cfg     窗长/保持参数
 * @retval 1 = 本拍处于"双击事件"保持窗内（事件沿后保持 hold_ticks 拍）
 * @note   判定=两次按下沿间隔 ≤window_ms；超窗的第二按视为新的第一按。
 *         单击不产生任何输出（将来若要单击语义另加通道）。
 */
uint8_t RcInput_DclickStep(RcInput_DclickState_t *st, uint8_t level,
                           uint32_t now_ms, const RcInput_DclickCfg_t *cfg);

/* 双击绑定条目（remoter_app 绑定表用）：某路消抖位发生双击 → flags 对应位 */
typedef struct
{
    uint16_t level_mask;    /* 消抖字位掩码（RCINPUT_KEY_K*_A/B 之一） */
    uint8_t  flag_index;    /* 事件写 flags 的位序号 1..6（bit0=estop 不可占） */
} RcInput_DclickBind_t;

/**
 * @brief  由输入快照构建一帧协议数据（整帧清零再填，"发送端纪律"唯一收口点）
 * @param  cfg   标定配置（中点/死区/关节速率）
 * @param  raw   本拍输入快照
 * @param  seq   发送序号（调用方维护，本函数透传不自增）
 * @param  out   输出帧，不可为 NULL
 */
void RcInput_Build(const RcInput_Config_t *cfg, const RcInput_Raw_t *raw,
                   uint8_t seq, RCPayload_t *out);

#endif /* RC_INPUT_H */
