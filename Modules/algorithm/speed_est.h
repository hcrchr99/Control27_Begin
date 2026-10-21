/**
 * @file    speed_est.h
 * @brief   M/T 测速（mt-speed-pending S-A）：边沿吸附窗，低速不再有量化台阶
 *
 * 纯算法（零硬件依赖、零动态分配、时钟由调用方传入）：吃一串"编码器边沿
 * 采样"（累计计数 + 微秒时间戳），测速窗两端吸附到真实边沿——分数计数不再
 * 被丢在窗尾，而是全部折进 Δt。测速误差从 M 法的"±1 计数/窗"（低速横移时
 * 一个台阶就是 1.2rpm）降到微秒级时间戳噪声，且每个控制拍都能出新鲜读数。
 * 为麦轮低速横移"一下一下"一案而生：辊子触地扰动在 2~5Hz，M 法 20ms 窗
 * 在 <15rpm 时又晚又糊，环路根本看不见扰动。
 *
 * 边界（对照 lpf.h 的纯算法定位）：
 *  - 不碰硬件：样本从哪来（边沿中断的环形缓冲）是 Bsp 的事，本模块只认数组；
 *  - 符号只到 counts 语义为止：duty 语义的方向修正（ROBOT_MOTOR_SIGN）仍归
 *    消费者（motor.c），与旧 M 法同口径，负转速原样返回负数；
 *  - 跨调用契约：每次调用喂的样本按时间升序、且不早于上次喂过的最新样本；
 *    now_us 不早于任何喂入样本的时间戳。
 */
#ifndef SPEED_EST_H
#define SPEED_EST_H

#include <stdbool.h>
#include <stdint.h>

/* 一个边沿采样：cnt = 16 位计数回绕展开后的累计计数（展开在采样侧做，这里
 * 只管差分），us = 该边沿时刻的微秒时间戳（uint32 约 71.6 分钟回绕，两时间
 * 戳直接相减跨回绕天然正确） */
typedef struct
{
    int32_t  cnt;   /* 上电以来的累计计数（已回绕展开） */
    uint32_t us;    /* 该边沿发生的时刻（µs） */
} SpeedEst_Sample_t;

typedef struct
{
    float    ppr;         /* 每输出圈计数（线数×4×减速比，robot_config 的 PPR 口径） */
    uint32_t win_min_us;  /* 最小窗龄：窗从最新样本往回找至少这么老的样本；窗越短越
                           * 新鲜但越容易吃到编码器刻线不均的抖，0 = 退化为最近两样本 */
    uint32_t stale_us;    /* 静止超时：最新样本距今超过它 → 判停归 0（边沿稀到没有
                           * 信息就不硬编）；0 = 只有完全同拍的样本不算停 */
    float    lpf_rc_us;   /* 输出低通时间常数 µs（抑制廉价码盘刻线不均的单间隔抖动）；
                           * 0 = 关，读数逐位透传 */
} SpeedEst_Config_t;

typedef struct
{
    SpeedEst_Config_t cfg;
    SpeedEst_Sample_t newest;   /* 最近一次喂到的最新样本（跨调用攒窗用） */
    bool              have_newest; /* 还没喂过任何样本时 newest 无效 */
    float             out;      /* 上次输出 rpm（counts 语义） */
    bool              primed;   /* 低通已喂过首值（首值直通，不从 0 爬坡）；判停后清零 */
    uint32_t          last_upd_us; /* 上次 Update 的 now_us（算低通步长用） */
    /* 病灶记录仪（板验 bisect）：stale 每次触发都留痕——年龄若远超喂送
     * 周期，就是时间戳/喂送链的病，数字本身就是诊断 */
    uint32_t          dbg_stale_age_us; /* 最近一次 stale 时的样本年龄 µs */
    uint32_t          dbg_stale_len;    /* 当时滴喂历史长度 */
    uint32_t          dbg_stale_n;      /* 当时本拍新样本数 */
    uint32_t          dbg_stale_cnt;    /* 累计触发次数 */
} SpeedEst_t;

/**
 * @brief  初始化：装参数（ppr 小于 1 拉回 1），清全部历史
 * @param  se  实例（调用方静态分配）
 * @param  cfg 参数；参考起点：win_min_us=4000、stale_us=30000、lpf_rc_us=0（关）
 */
void SpeedEst_Init(SpeedEst_t *se, const SpeedEst_Config_t *cfg);

/**
 * @brief  喂样本并取最新转速
 * @param  se      实例
 * @param  samples 本拍新增的边沿样本（按时间升序；一拍没有新边沿就传 NULL/0）
 * @param  n       样本个数
 * @param  now_us  当前时刻（判停与低通步长都用它）
 * @retval 转速 rpm（counts 语义）。本拍没有新边沿时保持上次读数（两个边沿
 *         之间信息量为零，保持就是最诚实的估计）；窗还没攒够 win_min 也保持；
 *         判停返回 0。只在"算出了新读数"时才过输出低通，保持的旧值不复滤
 * @note   永不返回 NaN/INF：分母只有 ppr（Init 兜底）和 Δt（0 时走保持分支）
 */
float SpeedEst_Update(SpeedEst_t *se, const SpeedEst_Sample_t *samples,
                      uint32_t n, uint32_t now_us);

/* ---------------- 滴喂包装（1kHz 消费者的标配入口） ----------------
 * 教训（2026-10-21 S-C 板验翻车案）：Update 本体跨调用只记"上一批的最新
 * 样本"，算一次要凑一对年龄差 ≥win_min 的样本。55rpm 时边沿每 874µs 一个、
 * 1kHz 喂一次只来一两滴，窗永远凑不齐 → 估计器抱住旧读数，速度环追着冻结
 * 值乱撞。包装在自己肚子里留一段滚动历史（跨度超 2×win_min 从最老端裁），
 * 每拍把整段窗递给本体——任何转速下每拍都有新鲜读数。长批直喂（整定台
 * 批量转储类）可以不用包装，直接 Update。 */
#define SPEED_EST_DRIP_DEPTH    32u     /* 8ms 窗 @3.1kHz 满速 = 25 样本，32 留余量 */

typedef struct
{
    SpeedEst_Sample_t hist[SPEED_EST_DRIP_DEPTH];   /* 滚动历史（最老在前） */
    uint16_t len;                                   /* 有效样本数 */
} SpeedEstDrip_t;

/**
 * @brief  清空滴喂历史（估计器本体另用 SpeedEst_Init 重装）
 */
void SpeedEstDrip_Init(SpeedEstDrip_t *dp);

/**
 * @brief  滴喂：并入本拍新样本 → 按窗龄裁历史 → 整段递给 Update
 * @param  dp      滴喂历史（调用方静态分配，每通道一个）
 * @param  se      估计器本体
 * @param  samples 本拍新样本（升序，可 NULL/0）
 * @param  n       本拍样本数
 * @param  now_us  当前时刻
 * @retval 同 SpeedEst_Update。停喂超 stale 后恢复的头几拍，窗会被裁剪
 *         压到 2×win_min 内，几毫秒即回到量程，不会拿跨停喂段的窗硬算
 */
float SpeedEstDrip_Update(SpeedEstDrip_t *dp, SpeedEst_t *se,
                          const SpeedEst_Sample_t *samples, uint32_t n,
                          uint32_t now_us);

#endif /* SPEED_EST_H */
