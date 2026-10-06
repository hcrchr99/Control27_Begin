/**
 * @file    bsp_encoder.h
 * @brief   四路正交编码器：TI12 双边沿计数，只出原始增量
 *
 * 设计边界（BSP层开发规划 v1.1 四.2 / 实例化边界分析）：
 *  - "×4 同构但归属固定"的多实例，用 ch 索引 + 内部静态表实现
 *    （编译期实例化），不引入 Register/Instance 框架；
 *  - 换算系数（线数×4、减速比）在 robot_config.h 的 ROBOT_ENC_PPR，
 *    本模块只出 16bit 回绕差分后的原始增量，不做物理量换算。
 */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

/* 通道宏：与 bsp_pin.h 的 PIN_ENC1~4_TIM 一一对应 */
#define ENC_CH1     1u  /* TIM1  PA8/PA9          */
#define ENC_CH2     2u  /* TIM2  PA15/PB3 */
#define ENC_CH3     3u  /* TIM3  PB4/PB5（2026-09-28 由 PA6/PA7 迁入） */
#define ENC_CH4     4u  /* TIM8  PC6/PC7          */

/**
 * @brief  启动全部四路编码器计数（MX_TIMx_Init 已完成编码器模式配置，
 *         本函数只负责 HAL_TIM_Encoder_Start；须在 main.c 外设初始化之后调用）
 */
void Encoder_InitAll(void);

/**
 * @brief  读取指定通道自上次调用以来的增量（16bit 回绕差分，int32 承载）
 * @param  ch ENC_CH1..ENC_CH4；非法值返回 0
 * @retval 增量计数，正 = A 相超前 B 相方向，符号随接线/转向，由上层标定
 *
 * 使用约束（BSP 不加锁，靠契约）：
 *  - 同一通道只允许一个读者任务（内部有 last 快照，多读者会互相吃掉增量）；
 *  - 相邻两次读取间隔内单通道增量不得超过 ±32767（16bit 计数窗口），
 *    1kHz 任务周期 + 168MHz 四倍频下满速计数余量充足（16bit 窗 ±32767）；
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

#endif /* BSP_ENCODER_H */
