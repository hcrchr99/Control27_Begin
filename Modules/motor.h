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
 *         + 速度环装参清态（PID 参数来自 robot_config.h，改参重跑本函数生效）
 * @retval true 恒成功（通道映射编译期定死，运行期无失败源）
 * @note   STBY 低时 TB6612 全桥关断，SetDuty 只改输入电平不出力——
 *         上电安全顺序由调用方保证：Init →（可选 SetDuty/SetSpeedRpm）→ Enable。
 *         须在 Bsp_Init 之后调用（PID 记时取 Bsp_GetUs）
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

/* ==================== W2.3 速度环（追加只增不改；S5 开环语义不变） ====================
 * 归属（Modules 规划 §3.1 接口冻结表 / 架构 §数据流）：速度环算法在本模块，
 * 1kHz 节拍由 ChassisTask（业务态）或测试台 6（联验态）调用
 * Motor_SpeedLoopUpdate 驱动，任务层只喂 ref、不碰 PWM/PID。
 * 闭环符号约定与开环判读一致：正 ref = 正 duty 的转向（开环阶梯里
 * "O=一致"的方向），符号修正统一走 ROBOT_MOTOR_SIGN / 电机接线，本层
 * 不引入第二套符号。闭环前置条件：测试台 6 开环阶段四路符号交叉判读
 * 全 O（编码器符号 × 电机方向不一致 = 正反馈自激）。
 */

/**
 * @brief  设定转速（rpm）
 * @param  ch MOTOR_CH1..4；非法值忽略
 * @param  rpm 设定转速，正 = 正 duty 的转向；超 ROBOT_MOTOR_SPEED_MAX_RPM
 *         钳位；NaN 保留原值（失安全：速度指令坏数不生效）
 * @note   ref 初值 0（Motor_Init 清零）；Motor_Disable 同时清 ref 与积分态，
 *         重新给速度前电机保持静止。STBY 低（未 Enable）时 ref 只记账不出力
 */
void Motor_SetSpeedRpm(MotorCh_t ch, float rpm);

/**
 * @brief  最近一次测速窗折算的实测转速（rpm，已折算到 duty 语义符号）
 * @note   测速窗 = ROBOT_MOTOR_SPEED_CALC_MS（默认 10ms），本接口返回窗口
 *         平均值（100Hz 刷新），不读编码器——编码器唯一读者是
 *         Motor_SpeedLoopUpdate（bsp_encoder"每通道一个读者"契约），
 *         联验读数必须走本接口，直接 Encoder_Read 会抢走测速增量
 */
float Motor_GetSpeedRpm(MotorCh_t ch);

/**
 * @brief  最近一次施加的占空比（-1..+1，含符号；调试/整定观测用）
 * @note   读的是本模块记账值，不读寄存器；滑行/死区清零路径也如实记账
 */
float Motor_GetDuty(MotorCh_t ch);

/**
 * @brief  速度环 PID 的积分累计量（duty 量纲；整定观测用，看积分顶没顶限）
 */
float Motor_GetIout(MotorCh_t ch);

/**
 * @brief  运行时改速度环 PID 参数（整定台用；写全部四路，立即生效）
 * @note   不回写 robot_config.h——整定终值人工抄回配置（配置是上电缺省，
 *         运行时改动只活在本次上电）
 */
void Motor_SetTune(float kp, float ki, float kd);

/**
 * @brief  读当前速度环 PID 参数（整定台 SHOW 回显用）
 */
void Motor_GetTune(float *kp, float *ki, float *kd);

/**
 * @brief  速度环一步（测速累计 → PID → SetDuty），1kHz 调用
 * @note   调用前须 Motor_Init（装 PID 参数/清态）；禁止在中断里调用
 *         （PID 实例状态非重入）；STBY 低时照常计算不出力
 */
void Motor_SpeedLoopUpdate(void);

#endif /* MOTOR_H */
