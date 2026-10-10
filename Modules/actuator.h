/**
 * @file    actuator.h
 * @brief   执行器保护层（W2.4）：缓动斜坡 + 堵转判据 + SG90 热保护，
 *          包装 servo 器件层的状态机（每关节一份实例）
 *
 * 分层与边界（Modules 层开发规划 §3.3）：
 *  - 本模块只有"器件知识"：缓动行进、电流超限坐实堵转、供电时长超限
 *    报热——触发器在这里；"什么时候动作是业务安全的"只有 grab 知道，
 *    拍板时机（如 NeedsCooldown 后何时敢卸力掉矿）在业务层；
 *  - 电流获取走依赖注入（cfg.get_current_a 函数指针绑
 *    Power_GetJointCurrent(k)，NULL = 无采样通道 = 无堵转判据）——
 *    本模块不知道 power 存在，不新增模块间 include 例外；
 *  - 实例由调用方持有（W2.4 测试台 / W2.5 grab_task），本模块零静态
 *    状态，纯逻辑可 PC 单测；
 *  - 时间由调用方经 now_ms 注入（u32 毫秒，无符号差分抗回绕），
 *    本模块不自取时戳；
 *  - ⚠ 一切函数禁止在 ISR 里调用（坑#9：状态机/累计器非重入）。
 *
 * 状态机（SetTarget 也可从 RELEASED 重进 MOVING，Servo_SetAngle 自动
 * 重新上力）：
 *   IDLE --SetTarget--> MOVING（按 ease_dps 斜坡发脉冲；供电累计器累加）
 *   MOVING --斜坡计时到（纯时间比较，必然发生）--> HOLDING（恒发目标脉冲）
 *   HOLDING --SetTarget--> MOVING（重算斜坡）
 *   任意态 --Act_Release--> RELEASED（停脉冲卸力，停判停累计）
 *   MOVING/HOLDING --电流超阈值持续 stall_confirm_ms--> 立即卸力 +
 *                    IsStalled 锁存（业务可查）
 *   供电累计超 energize_max（仅使能实例）--> IsNeedsCooldown 置位
 *                    （只报告，不强制卸力；连续 cooldown_ms 未供电
 *                    自动解除并清累计器）
 *
 * ⚠ 无反馈本质：cur_deg 是"我们发出的最后角度"的开环记账，不是测量值。
 * RELEASED 期间外力改姿后再 SetTarget，斜坡起点即失真（业务侧注意，
 * 如重抓前小步试探）；参数默认值见 robot_config.h ROBOT_ACT_*。
 */
#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>
#include "servo.h"

/* 热保护滑动桶上限（1s 粒度）：供电累计窗口最长 30s（robot_config.h
 * ROBOT_ACT_ENERGIZE_WINDOW_MS），配置超出部分按 30s 生效（向更严侧收） */
#define ACT_BUCKET_MAX  30u

typedef enum
{
    ACT_IDLE = 0,       /* 已初始化待命：不发角度（舵机保持 servo 层中位） */
    ACT_MOVING,         /* 斜坡行进中：每拍按 ease_dps 步进发脉冲 */
    ACT_HOLDING,        /* 到位保持：恒发目标脉冲（供电累计继续） */
    ACT_RELEASED,       /* 已卸力：停脉冲，不判堵转不累计 */
} ActState_t;

typedef struct
{
    ServoId_t servo_id;             /* 驱动的舵机通道 */
    float     min_deg, max_deg;     /* 业务软限位（应比机械限位再收 5~10°；
                                       servo 层限位是最后一道钳位，双保险） */
    float     ease_dps;             /* 缓动限速（°/s，>0；见坑#12 双重斜坡纪律） */
    float   (*get_current_a)(void); /* 关节电流源（A），绑 Power_GetJointCurrent；
                                       NULL = 无采样通道 = 无堵转判据 */
    float     stall_current_a;      /* 堵转电流阈值（A；≤0 视为判据禁用） */
    uint32_t  stall_confirm_ms;     /* 超阈值确认时长（瞬时冲击不误触发） */
    uint32_t  energize_window_ms;   /* 热保护滑动窗（1s 粒度，≤ACT_BUCKET_MAX 秒） */
    uint32_t  energize_max_ms;      /* 窗内供电累计上限；0 = 热保护禁用 */
    uint32_t  cooldown_ms;          /* 热解除所需连续未供电时长 */
} ActConfig_t;

typedef struct
{
    bool        inited;
    ActConfig_t cfg;                /* Init 拷贝，之后与本模块外部无关 */

    ActState_t  state;
    float       cur_deg;            /* 开环记账：最后发出的角度（初值 90 = 中位） */
    float       target_deg;
    uint32_t    last_ms;            /* 最近 Update 时戳（差分基準） */
    bool        timed;              /* 首拍校准标志：第一拍只对表不出步 */
    uint32_t    t_move_ms;          /* 本次斜坡总时长 */
    uint32_t    move_start_ms;      /* 斜坡起点时戳 */

    bool        over_th;            /* 电流正处超阈值中 */
    uint32_t    over_start_ms;      /* 本轮超阈值起始（确认窗） */
    bool        stalled;            /* 堵转锁存（下个 SetTarget 清除） */

    uint16_t    bucket[ACT_BUCKET_MAX]; /* 1s 粒度供电累计（ms/桶），环形 */
    uint8_t     n_bucket;           /* 实际桶数 = 窗口秒数 */
    uint8_t     bucket_head;        /* 当前秒的桶下标 */
    uint32_t    bucket_slide_ms;    /* 上次滑桶时刻（1s 相位） */

    bool        hot;                /* NeedsCooldown 锁存 */
    bool        idle_t0_valid;      /* 冷却计时起点有效 */
    uint32_t    idle_t0_ms;         /* 连续未供电起始时刻 */
} Actuator_t;

/**
 * @brief  初始化实例（拷贝配置、清态、记中位 90° 为当前位置）
 * @retval false 参数非法：空指针 / servo_id 越界 / ease_dps≤0 / min≥max
 */
bool Act_Init(Actuator_t *a, const ActConfig_t *cfg);

/**
 * @brief  设定目标角度：钳进业务软限位、重算斜坡、进入/保持 MOVING
 * @param  deg 目标（°）；NaN 忽略
 * @note   同时清除 stalled 锁存与新动作堵转判定（重新起判）；斜坡时长
 *         基准 = 最近一次 Update 时戳，误差 ≤ 一个 Update 周期
 */
void Act_SetTarget(Actuator_t *a, float deg);

/**
 * @brief  周期驱动（10~20ms 节拍，调用方传入当前毫秒时戳）：斜坡步进/
 *         恒发保持、堵转判据、热累计，全在本拍内闭环
 * @note   首拍只对表校准（不出步不判），之后每拍正常
 */
void Act_Update(Actuator_t *a, uint32_t now_ms);

/**
 * @brief  卸力：停脉冲（PM10S/SG90 一致）转 RELEASED；堵转判定随供电
 *         停止复位，stalled 锁存保留到下个 SetTarget
 */
void Act_Release(Actuator_t *a);

/** @brief 到位保持中（MOVING 完成、恒发目标脉冲） */
bool Act_IsSettled(const Actuator_t *a);

/** @brief 堵转已坐实（锁存，下个 SetTarget 清除） */
bool Act_IsStalled(const Actuator_t *a);

/** @brief 热保护触发（锁存；连续 cooldown_ms 未供电自动解除） */
bool Act_NeedsCooldown(const Actuator_t *a);

#endif /* ACTUATOR_H */
