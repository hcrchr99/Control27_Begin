/**
 * @file    rc_cmd.h
 * @brief   RC_Cmd 遥控协议层（W2.2）：链路层 remote 之上的"指令语义"，
 *          本头文件的帧布局即协议冻结稿（两端同步演进，只修 bug 不改签名）
 *
 * 分层与边界（Modules 规划 §3.8 / 架构 §5.1）：
 *  - 对上暴露"已解析的定量指令"（归一化 -1.0..+1.0 的比例值，不是 m/s 或 °/s
 *    ——物理换算归 chassis/grab，本模块不知道电机/舵机/底盘的存在）；
 *  - 对下消费 remote 的 15B 字节流帧；车端 PRX 解析（remote_task 调
 *    Update/GetCopy），发送端 PTX/遥控器直接填 RCPayload_t 发包。
 *
 * 帧布局 v2（2026-10-06 与用户拍板冻结，15B == ROBOT_REMOTE_PAYLOAD）：
 *   [0]      uint8   seq       发送序号（uint8 回绕；丢包诊断用）
 *   [1..2]   int16   vx        逻辑量 -1000..+1000（发送端负责钳位）
 *   [3..4]   int16   vy        同上
 *   [5..6]   int16   omega     同上
 *   [7..11]  int8    joint[5]  速率指令 -100..+100；顺序 = 舵机 ID 序：
 *                              大臂/小臂/手腕/爪开合/爪旋转（id↔关节以实机布线为准）
 *   [12..13] uint16  keys      按键位图——位的含义不在 W2.2 冻结（按键→动作是 UserApp 业务）
 *   [14]     uint8   flags     bit0 = estop（RC_FLAG_ESTOP），发送端按住期间连发
 *  无应用层校验和：RF 层 Enhanced ShockBurst 自带 CRC16 + PID 去重，已把门。
 *  发送端纪律：每帧整帧清零再填（不给无线送垃圾字节）。两端同为小端
 *  Cortex-M，结构体整帧 memcpy 直传成立。
 *
 * 并发三行字（架构 §5.1，本模块的立身之本）：
 *  1. 唯一写者：remote_task（收到帧时调 Update）；看门狗超时是第二写者
 *     （remote_task 以 NULL 帧驱动 Update 写失效值）；
 *  2. 唯一读法：RC_Cmd_GetCopy——关调度内整结构体拷贝快照。禁止任何任务
 *     直读模块内状态：vx/vy/omega 三个 float 不是一次性写入的，任务切换
 *     插进来的话会抄到"半新半旧"的缝合数据；
 *  3. 故障语义两级：estop = 急停，指令立即归零并闩住（松键才解除）；
 *     失联 = valid 立即 false + 最后指令 300ms 线性滑落到零（渐停防甩矿，
 *     不是猛刹车）。应用层只看 valid / estop 两个位。
 *
 * 白名单登记（Modules 规划 §一.1"新用法须先在规划文档登记"通道）：
 *  rc_cmd.c 为实现关调度快照 include FreeRTOS.h/task.h（vTaskSuspendAll/
 *  xTaskResumeAll，架构 §5.1 处方）；PC 单测用 stub 头替身（同 bsp_sys 假时钟先例）。
 */
#ifndef RC_CMD_H
#define RC_CMD_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_config.h"

/* 急停标志：RCPayload_t.flags 的 bit0 */
#define RC_FLAG_ESTOP   0x01u

/* ==================== 无线帧布局（协议冻结稿，勿改字段） ==================== */

#pragma pack(1)
typedef struct
{
    uint8_t  seq;                                   /* 发送序号 */
    int16_t  vx;                                    /* -ROBOT_CMD_RANGE..+ROBOT_CMD_RANGE */
    int16_t  vy;
    int16_t  omega;
    int8_t   joint[ROBOT_CMD_JOINT_COUNT];          /* -ROBOT_CMD_JOINT_RANGE..+.. */
    uint16_t keys;                                  /* 按键位图（业务定义） */
    uint8_t  flags;                                 /* bit0=estop */
} RCPayload_t;                                      /* 15B */
#pragma pack()

/* 帧长必须与 RF 载荷严格一致：两端任何一端改了布局，这里立刻编译报错 */
_Static_assert(sizeof(RCPayload_t) == ROBOT_REMOTE_PAYLOAD,
               "RCPayload_t size must equal ROBOT_REMOTE_PAYLOAD (both ends)");

/* ==================== 解析后的指令快照（消费者唯一可见形态） ==================== */

typedef struct
{
    float vx;               /* -1.0..+1.0（÷ROBOT_CMD_RANGE 归一化，"比例油门"） */
    float vy;
    float omega;
    float joint[ROBOT_CMD_JOINT_COUNT];  /* -1.0..+1.0（÷ROBOT_CMD_JOINT_RANGE） */
    uint16_t keys;          /* 透传，位含义归 UserApp */
    uint8_t  seq;           /* 最新帧序号（丢包诊断） */
    bool valid;             /* true = 链路活着且数据新鲜 */
    bool estop;             /* true = 急停闩住（指令已强制归零） */
    uint32_t last_frame_ms; /* 最近一帧到达时刻（调用方时基） */
} RC_Cmd_t;

/**
 * @brief  清态：全部指令归零、valid/estop=false、计数清零
 * @note   任务启动时调一次；之后任意时刻可重调（重置回安全态）
 */
void RC_Cmd_Init(void);

/**
 * @brief  喂一帧（唯一写者 = remote_task）
 * @param  f      收到的帧；NULL = 看门狗空帧驱动（失联路径）
 * @param  now_ms 当前毫秒时基（调用方递入——函数不含时间源，PC 单测直接喂假时间）
 * @note   f 非 NULL：归一化存值、valid=true。estop 位=1 → 指令立即归零 +
 *         estop 闩住，直到 estop=0 的帧到达才解除。
 *         f 为 NULL（仅应在 Remote_IsLinkUp()==false 时调）：valid 立即
 *         false，最后指令从本时刻起 ROBOT_CMD_DECAY_MS 内线性滑落到零，
 *         之后恒零；estop 标志保持不变。
 */
void RC_Cmd_Update(const RCPayload_t *f, uint32_t now_ms);

/**
 * @brief  唯一读法：关调度内整结构体快照（撕裂免疫）
 * @param  out 快照输出（调用方栈上变量即可）
 * @retval true = 已拷贝（数据可信度看 out->valid / out->estop）；false = out 为 NULL
 * @note   关调度仅持续一次 40B 拷贝（微秒级）；禁止在中断里调用
 *         （vTaskSuspendAll 仅限任务上下文，亦同 Modules ISR 禁则）
 */
bool RC_Cmd_GetCopy(RC_Cmd_t *out);

#endif /* RC_CMD_H */
