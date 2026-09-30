/**
 * @file    bsp_pwm.h
 * @brief   PWM 输出：TIM4×4 20kHz 占空比 / TIM5×4 50Hz 脉宽，统一 ch 接口
 *
 * 设计边界（BSP层开发规划 v1.1 四.1 / 实例化边界分析）：
 *  - 两组"×N 同构、归属固定"的多实例，用 ch 索引 + 内部静态表实现
 *    （编译期实例化），不引入 Register/Instance 框架；
 *  - 纯片上外设接口，不知道"电机/舵机"为何物：占空比按各通道所属
 *    定时器的 ARR 查表换算，脉宽接口只对 50Hz 组开放（对 20kHz 电机
 *    通道按 µs 设脉宽没有物理意义）；
 *  - 上电安全态 = CubeMX Pulse=0（duty=0 / 不发脉冲=舵机卸力），Init
 *    不改写任何 compare，安全链不依赖上层调用顺序；舵机中位脉冲由
 *    Modules/servo 的 Init 负责（500~2500µs 量程是舵机知识，不进 BSP）。
 */
#ifndef F103RC_BSP_PWM_H
#define F103RC_BSP_PWM_H

#include <stdint.h>

/* 通道宏：统一编号空间，与 bsp_pin.h 的 PIN_PWM20K_* / PIN_PWM50HZ_* 一一对应 */
#define PWM_20K_CH1     1u  /* TIM4 CH1  PB6 */
#define PWM_20K_CH2     2u  /* TIM4 CH2  PB7 */
#define PWM_20K_CH3     3u  /* TIM4 CH3  PB8 */
#define PWM_20K_CH4     4u  /* TIM4 CH4  PB9 */
#define PWM_50HZ_CH1    5u  /* TIM5 CH1  PA0 */
#define PWM_50HZ_CH2    6u  /* TIM5 CH2  PA1 */
#define PWM_50HZ_CH3    7u  /* TIM5 CH3  PA2 */
#define PWM_50HZ_CH4    8u  /* TIM5 CH4  PA3 */

/**
 * @brief  启动全部 8 路 PWM 输出（MX_TIM4/5_Init 已完成时基与通道配置，
 *         本函数只负责 HAL_TIM_PWM_Start；须在 main.c 外设初始化之后调用）
 * @note   刻意不写任何 compare：保持 CubeMX 的 Pulse=0 上电安全态。
 */
void Pwm_InitAll(void);

/**
 * @brief  设置指定通道占空比（全部 8 通道可用）
 * @param  ch PWM_20K_CH1..4 / PWM_50HZ_CH1..4；非法值忽略
 * @param  duty 0.0f..1.0f；<0 钳 0（PWM 无方向概念，负值视为非法输入
 *         失安全停输出，方向语义归 Modules 层）、>1 钳 1、NaN 落 0
 *         （NaN 参与比较恒 false，被 <0 分支天然吞掉）
 *
 * 换算：CCR = duty × (所属定时器 ARR+1)，四舍五入取整；duty=1.0 时
 * CCR=ARR+1，PWM1 模式下 CNT<CCR 全程成立，即 100% 占空比。
 * F103 无 FPU，float 走软浮点（数十 cycle），控制环调用频率下无感；
 * 若未来要在高频 ISR 里调本接口，应换定点。
 *
 * 使用约束（BSP 不加锁，靠契约）：同通道归 Modules 单一属主，多任务
 * 同写同通道不保证最后写入者。
 */
void Pwm_SetDuty(uint8_t ch, float duty);

/**
 * @brief  设置指定通道脉宽（仅 50Hz 组 TIM5 CH1-4，其余通道忽略）
 * @param  ch PWM_50HZ_CH1..4
 * @param  us 正脉宽，0..20000µs；超界钳到 20000（= 1000 步 = 恒高）。
 *         1 步 = 20µs（PIN_PWM50HZ_US_PER_STEP）；500~2500µs 舵机软限位
 *         是舵机知识，由 Modules/servo 负责钳位，BSP 只保物理可行范围
 */
void Pwm_SetPulseUs(uint8_t ch, uint32_t us);

/**
 * @brief  释放指定通道到安全态（duty=0 / 不发脉冲）
 * @param  ch PWM_20K_CH1..4 / PWM_50HZ_CH1..4；非法值忽略
 * @note   实现为 compare 写 0 而非 HAL_TIM_PWM_Stop：PWM1 模式下 CCR=0
 *         输出恒低、无脉冲边沿，等效停发，且可被 SetDuty/SetPulseUs
 *         直接恢复，不引入通道启停状态机。
 */
void Pwm_Release(uint8_t ch);

#endif /* F103RC_BSP_PWM_H */
