/**
 * @file    motor.h
 * @brief   TB6612 ×4 直流电机：方向 + 占空比（S5 开环；速度环 W2 随 algorithm 追加）
 *
 * 分层与边界（Modules 层开发规划 §3.1）：
 *  - 封装器件知识：TB6612 真值表、符号→方向映射、死区、通道映射
 *    （spec 对照清单见 motor.c 头部）；对上只暴露"设占空比/使能"设备能力，
 *    不知道底盘/轮子等业务概念；
 *  - 对下只消费 bsp_pwm（TIM4 20kHz 占空比）与 bsp_gpio（DIR ×8 + STBY）；
 *  - 上电安全链：CubeMX 复位 STBY=低（关断，gpio.c 核实）→ Motor_Init 保持低
 *    → Motor_Enable 才出力；Motor_Disable = duty 0 + STBY 低双保险。
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_CH1 = 0,
    MOTOR_CH2,
    MOTOR_CH3,
    MOTOR_CH4,
    MOTOR_CH_COUNT,
} MotorCh_t;

/**
 * @brief  初始化：启动 PWM（幂等）+ 全通道 DIR 复位 + duty 0 + STBY 保持低
 * @retval true 恒成功（通道映射编译期定死，运行期无失败源）
 * @note   STBY 低时 TB6612 全桥关断，SetDuty 只改输入电平不出力——
 *         上电安全顺序由调用方保证：Init →（可选 SetDuty）→ Enable。
 */
bool Motor_Init(void);

/**
 * @brief  设置开环占空比
 * @param  ch MOTOR_CH1..4；非法值忽略
 * @param  duty -1.0f..+1.0f，符号即方向（× ROBOT_MOTOR_SIGN 修正轮向）；
 *         超界钳位；|duty| < ROBOT_MOTOR_DEADBAND 按 0 处理；NaN 滑行
 *         （失安全：NaN 混过钳位会置 DIR 而 PWM=0，真值表=短刹，显式拦截）
 * @note   与 Pwm_SetDuty 同量纲（2026-10-03 变更：int16 ±1000 → float）——
 *         W2 速度环 PID 的 MaxOut=1.0f 输出可直喂本接口，链路零换算。
 *         同边沿切换：先写 DIR 再写 PWM，杜绝共导通。
 *         duty=0 → IN=00 滑行（STOP，输出高阻，手转自由）；
 *         PWM 斩波低电平相 = 短刹，占空比即平均电压（驱动/短刹斩波），
 *         主动刹停语义由上层 Disable/短刹承接（W2 速度环再定）。
 */
void Motor_SetDuty(MotorCh_t ch, float duty);

/** @brief 使能功率级（STBY 拉高；Init 后默认关断，可重复调用） */
void Motor_Enable(void);

/**
 * @brief  关断功率级：全通道 duty 0（IN=00）+ STBY 拉低双保险
 * @note   STBY 低优先级最高（真值表首行），输出关断 = 手转轮无阻力矩
 */
void Motor_Disable(void);

/* —— W2 速度环（algorithm 就绪后追加，追加只增不改；S5 不实现）——
 * void  Motor_SetSpeedRpm(MotorCh_t ch, float rpm);
 * float Motor_GetSpeedRpm(MotorCh_t ch);   （Encoder_Read × ROBOT_ENC_PPR 换算）
 */

#endif /* MOTOR_H */
