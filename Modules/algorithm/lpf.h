/**
 * @file    lpf.h
 * @brief   一阶低通滤波（W2.1 自研轻量件）：抹平摇杆/采样数据里的抖动
 *
 * 纯算法（Modules 层开发规划 §3.7）：零硬件依赖、零动态分配、不带时间
 * 概念——平滑系数 alpha 由调用方按自己的调用频率定好。alpha 越小输出越
 * 平滑、但对真实变化的反应越迟钝；alpha=1 等于不滤波，alpha=0 输出纹丝不动。
 */
#ifndef LPF_H
#define LPF_H

#include <stdbool.h>

typedef struct
{
    float alpha;    /* 平滑系数 0..1：越小越平滑越迟钝（Init 时超出范围会拉回来） */
    float out;      /* 上次的输出 */
    bool  primed;   /* 是否已经喂过第一个数（第一个数直接通过，不从 0 爬坡） */
} Lpf_t;

/**
 * @brief  初始化：装平滑系数（超出 0..1 拉回边界），清历史输出
 * @param  f     滤波器实例（调用方静态分配）
 * @param  alpha 平滑系数：越小越平滑越迟钝，越大越灵敏
 */
void Lpf_Init(Lpf_t *f, float alpha);

/**
 * @brief  喂进一个新样本，返回平滑后的输出
 * @param  f  滤波器实例
 * @param  in 输入样本
 * @retval 第一个样本原样返回（不从 0 慢慢爬向它，避免上电假瞬态）；
 *         之后 out = out + (in - out) * alpha，即向新数据靠一小步
 * @note   输入若混进 NaN 这类坏数会一直留在输出里，本模块不清洗——
 *         有坏数风险的数据源在上游挡住
 */
float Lpf_Apply(Lpf_t *f, float in);

#endif /* LPF_H */
