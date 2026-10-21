/**
 * @file    bsp_encoder.h
 * @brief   四路正交编码器：TI12 双边沿计数 + 边沿时间戳采样环（M/T 测速用）
 *
 * 设计边界（BSP层开发规划 v1.1 四.2 / 实例化边界分析）：
 *  - "×4 同构但归属固定"的多实例，用 ch 索引 + 内部静态表实现
 *    （编译期实例化），不引入 Register/Instance 框架；
 *  - 换算系数（线数×4、减速比）在 robot_config.h 的 ROBOT_ENC_PPR，
 *    本模块只出原始计数，不做物理量换算。
 *
 * 两套并存的读取路径（mt-speed-pending S-B 起）：
 *  1. 直读（Encoder_Read / Encoder_GetCount）：被动读 CNT 寄存器、各自维护
 *     快照，语义与历史版本完全一致——烟雾测试/整定台等慢轮询消费者继续用，
 *     不会被采样环抢走增量（读 CNT 是旁路观察，不吃任何人的账）；
 *  2. 采样环（Encoder_TakeNew）：每路边沿中断把"16 位计数回绕展开后的累计
 *     计数 + 微秒时间戳"推入 16 格环形缓冲，专供速度环的 M/T 测速消费。
 *     中断是环的唯一写者、也是唯一成对的 CNT 记账者；每通道只允许一个
 *     TakeNew 消费者（游标在模块内，调用方不持有）。
 *
 * 边沿采样脚位（⚠ CH3 特例）：CH1=PA8、CH2=PA15、CH4=PC6 均抓 A 相；
 * CH3 抓 PA7（B 相）——PA6 与 PC6 同为 EXTI 线 6 不可共存，B 相边沿率与
 * A 相相同（每输出圈各 1248 沿），对测速完全等价。四线占用 8/15/7/6，
 * 与按键/无线的 10/11/12 不冲突（PA15 与它们同挂 EXTI15_10 向量）。
 */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/* 通道宏：与 bsp_pin.h 的 PIN_ENC1~4_TIM 一一对应 */
#define ENC_CH1     1u  /* TIM1  PA8/PA9          */
#define ENC_CH2     2u  /* TIM2  PA15/PB3 */
#define ENC_CH3     3u  /* TIM3  PA6/PA7（2026-10-15 测试期由 PB4/PB5 迁回，实车 PB4/PB5 不变） */
#define ENC_CH4     4u  /* TIM8  PC6/PC7          */

/* 采样环深度：1kHz 消费下按最坏边沿率留余量——满速 150rpm = 3.1kHz 边沿，
 * 64 格 ≈ 22.8ms，喂送侧偶发十几毫秒的阻塞（日志打印等）不再击穿；
 * （16 格时代 135rpm 只有 5.7ms 余量，S-C 板验曾被日志打印打穿，教训在案） */
#define ENC_RING_DEPTH  64u

/* 一个边沿采样：cnt = 16 位计数回绕展开后的累计计数（中断里展开），
 * us = 该边沿时刻的微秒时间戳（Bsp_GetUs，uint32 回绕约 71.6 分钟） */
typedef struct
{
    int32_t  cnt;
    uint32_t us;
} Encoder_Sample_t;

/**
 * @brief  启动全部四路编码器计数并打开边沿采样（MX_TIMx_Init 已完成编码器
 *         模式配置，本函数负责 HAL_TIM_Encoder_Start + EXTI 手配 + 回调注册；
 *         须在 main.c 外设初始化之后调用）
 * @note   EXTI 配置直接写寄存器（EXTICR/IMR/RTSR/FTSR），不走 HAL_GPIO_Init
 *         的 EXTI 模式——那会把引脚 MODER 从复用改回输入，编码器计数当场断；
 *         引脚保持 TIM 复用态，EXTI 与 TIM 并联采样同一焊盘
 */
void Encoder_InitAll(void);

/**
 * @brief  读取指定通道自上次调用以来的增量（16bit 回绕差分，int32 承载）
 * @param  ch ENC_CH1..ENC_CH4；非法值返回 0
 * @retval 增量计数，正 = A 相超前 B 相方向，符号随接线/转向，由上层标定
 *
 * 使用约束（BSP 不加锁，靠契约）：
 *  - 与采样环并存：本接口是被动旁路读，与边沿中断互不干扰；
 *  - 多个消费者仍禁止：本接口内部有 last 快照，两处调用会互相吃掉增量
 *    （采样环的 TakeNew 不受此限，见其注释）；
 *  - 相邻两次读取间隔内单通道增量不得超过 ±32767（16bit 计数窗口），
 *    1kHz 任务周期 + 168MHz 四倍频下满速计数余量充足；
 *  - CNT 为 16bit 寄存器，单次读取原子，无需临界区。
 */
int32_t Encoder_Read(uint8_t ch);

/**
 * @brief  读取指定通道上电以来的累计计数（16bit 回绕补偿后的绝对计数）
 * @param  ch ENC_CH1..ENC_CH4；非法值返回 0
 * @note   CNT 寄存器只有 16 位（会回绕），不能直接当绝对位置读；
 *         本接口内部累计，int32 上限 ±21.4 亿计数（4096 计数/圈 ≈ ±52 万圈）。
 *         与 Encoder_Read 各自维护独立快照，可并存调用、互不干扰，
 *         但同一接口仍遵守"同一通道一个读者任务"的约束。
 */
int32_t Encoder_GetCount(uint8_t ch);

/**
 * @brief  通道边沿产出计数（单调递增写序号快照，uint16 回绕差分安全）
 * @param  ch ENC_CH1..ENC_CH4；非法值返回 0
 * @note   板验 bisect 仪器：与 Encoder_TakeNew 的消费量对照——环产出在涨
 *         而读数为 0 = 消费/估计侧问题（软件）；产出也停了 = 编码器/EXTI
 *         静默（硬件接线）；未在转的通道产出疯涨 = 噪声假边沿风暴（EMI）
 */
uint16_t Encoder_Head(uint8_t ch);

/**
 * @brief  取走采样环中自上次调用以来的新样本（速度环 M/T 测速专用）
 * @param  ch      ENC_CH1..ENC_CH4；非法值返回 0
 * @param  out     样本落点（按时间升序填入）
 * @param  max     out 容量；给到 ENC_RING_DEPTH 即任何情况都不丢样
 * @param  dropped 返回"消费者掉队超一圈、老样本被覆盖"的标志（true = 样本
 *                 流不连续，消费方应复位自己的滤波/估计状态）；
 *                 传 NULL 忽略
 * @retval 实际拷贝的样本数（可能小于 max——没攒够新样本；也可能小于掉队
 *         后的应得数——容量截断，余下样本下次调用继续给）
 * @note   仅限任务上下文调用；每通道只允许一个消费者（游标在模块内）。
 *         极端时序下掉队批内的最老样本可能与新边沿撕裂（cnt/us 错配），
 *         但 dropped=true 已声明不连续，消费方复位后该批即弃用
 */
uint8_t Encoder_TakeNew(uint8_t ch, Encoder_Sample_t *out, uint8_t max, bool *dropped);

#endif /* BSP_ENCODER_H */
