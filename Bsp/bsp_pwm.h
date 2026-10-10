/**
 * @file    bsp_pwm.h
 * @brief   PWM 输出：TIM4×4 20kHz / TIM9×2 250Hz / TIM5×3 50Hz /
 *          TIM14×1 蜂鸣器方波，统一 ch 接口
 *
 * 设计边界（BSP层开发规划 v1.1 四.1 / 实例化边界分析）：
 *  - 各组"×N 同构、归属固定"的多实例，用 ch 索引 + 内部静态表实现
 *    （编译期实例化），不引入 Register/Instance 框架；
 *  - 纯片上外设接口，不知道"电机/舵机/蜂鸣器"为何物：占空比按各通道
 *    所属定时器的 ARR 查表换算，脉宽接口只对 250Hz/50Hz 两组开放（对
 *    20kHz 电机通道按 µs 设脉宽没有物理意义，蜂鸣器组只管占空比响停）；
 *  - 2026-10-05 F407VG 迁移：舵机按帧率分组（不同帧率不能共 TIM，Modules
 *    规划 v1.1 预埋落地）——PM10S 数字舵机走 TIM9 250Hz，SG90 走 TIM5 50Hz；
 *  - 2026-10-10：新增 TIM14 组（无源蜂鸣器方波激励，响=50% duty/停=0，
 *    音调频率由 TIM14 PSC/ARR 的 .ioc 时基决定）；
 *  - 上电安全态 = CubeMX Pulse=0（duty=0 / 不发脉冲=舵机卸力 / 蜂鸣器静音），
 *    Init 不改写任何 compare，安全链不依赖上层调用顺序；舵机中位脉冲由
 *    Modules/servo 的 Init 负责（500~2500µs 量程是舵机知识，不进 BSP）。
 */
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

/* 通道宏：统一编号空间，与 bsp_pin.h 的 PIN_PWM20K_* / PIN_PWM250HZ_* /
 * PIN_PWM50HZ_* / PIN_PWM14_* 一一对应。编号 = 组序（20K → 250Hz → 50Hz →
 * 蜂鸣器）依次连排 */
#define PWM_20K_CH1     1u  /* TIM4 CH1  PB6 */
#define PWM_20K_CH2     2u  /* TIM4 CH2  PB7 */
#define PWM_20K_CH3     3u  /* TIM4 CH3  PB8 */
#define PWM_20K_CH4     4u  /* TIM4 CH4  PB9 */
#define PWM_250HZ_CH1   5u  /* TIM9 CH1  PE5（大臂/小臂 PM10S 组，250Hz） */
#define PWM_250HZ_CH2   6u  /* TIM9 CH2  PE6 */
#define PWM_50HZ_CH1    7u  /* TIM5 CH1  PA0（手腕/爪 SG90 组，50Hz） */
#define PWM_50HZ_CH2    8u  /* TIM5 CH2  PA1 */
#define PWM_50HZ_CH3    9u  /* TIM5 CH3  PA2 */
#define PWM_TIM14_CH1   10u /* TIM14 CH1 PA7（无源蜂鸣器方波组，响停占空比控制） */

/**
 * @brief  启动全部 10 路 PWM 输出（MX_TIM4/9/5/14_Init 已完成时基与通道
 *         配置，本函数只负责 HAL_TIM_PWM_Start；须在 main.c 外设初始化之后
 *         调用）
 * @note   刻意不写任何 compare：保持 CubeMX 的 Pulse=0 上电安全态。
 */
void Pwm_InitAll(void);

/**
 * @brief  设置指定通道占空比（全部 10 通道可用）
 * @param  ch PWM_20K_CH1..4 / PWM_250HZ_CH1..2 / PWM_50HZ_CH1..3 /
 *         PWM_TIM14_CH1；非法值忽略
 * @param  duty 0.0f..1.0f；<0 钳 0（PWM 无方向概念，负值视为非法输入
 *         失安全停输出，方向语义归 Modules 层）、>1 钳 1、NaN 落 0
 *         （NaN 参与比较恒 false，被 <0 分支天然吞掉）
 *
 * 换算：CCR = duty × (所属定时器 ARR+1)，四舍五入取整；duty=1.0 时
 * CCR=ARR+1，PWM1 模式下 CNT<CCR 全程成立，即 100% 占空比。
 * F407 带 FPU（hard ABI），float 运算硬浮点，控制环调用频率下无感。
 *
 * 使用约束（BSP 不加锁，靠契约）：同通道归 Modules 单一属主，多任务
 * 同写同通道不保证最后写入者。
 */
void Pwm_SetDuty(uint8_t ch, float duty);

/**
 * @brief  设置指定通道脉宽（仅 250Hz 组 TIM9 CH1-2 与 50Hz 组 TIM5 CH1-3，
 *         其余通道忽略）
 * @param  ch PWM_250HZ_CH1..2 / PWM_50HZ_CH1..3
 * @param  us 正脉宽，按所属组钳位：250Hz 组 0..4000µs（= 2000 步 = 恒高，
 *         4ms 帧长上限）；50Hz 组 0..20000µs（= 10000 步 = 恒高，20ms 帧）。
 *         两组均为 1 步 = 2µs（PIN_PWM250HZ_/PIN_PWM50HZ_US_PER_STEP）；
 *         500~2500µs 舵机软限位是舵机知识，由 Modules/servo 负责钳位，
 *         BSP 只保物理可行范围（脉宽 < 帧长）
 */
void Pwm_SetPulseUs(uint8_t ch, uint32_t us);

/**
 * @brief  释放指定通道到安全态（duty=0 / 不发脉冲）
 * @param  ch PWM_20K_CH1..4 / PWM_250HZ_CH1..2 / PWM_50HZ_CH1..3 /
 *         PWM_TIM14_CH1；非法值忽略
 * @note   实现为 compare 写 0 而非 HAL_TIM_PWM_Stop：PWM1 模式下 CCR=0
 *         输出恒低、无脉冲边沿，等效停发，且可被 SetDuty/SetPulseUs
 *         直接恢复，不引入通道启停状态机。
 */
void Pwm_Release(uint8_t ch);

#endif /* BSP_PWM_H */
