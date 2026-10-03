/**
 * @file    servo.h
 * @brief   舵机 ×4（TIM5 50Hz 脉宽）：脉宽↔角度、软限位、卸力（S5 器件层）
 *
 * 分层与边界（Modules 层开发规划 §3.2）：
 *  - 本模块只回答"怎么动"（器件知识：脉宽↔角度、软限位、卸力），不知道
 *    "何时动、动多久"——那是 actuator（W2.4）包装的状态机，再上是 grab；
 *  - 器件分型（2026-10-01 定案，robot_config.h ROBOT_SERVO_*_LIST 配置）：
 *    · 数字舵机（ID1 大臂 / ID2 小臂，PM10S 级）= fail-hold：发一次脉冲即
 *      锁存保持，停发脉冲 ≠ 卸力（内部闭环照常出力）；软件无卸力手段，
 *      真卸力需供电断开（硬件 MOS 开关，转硬件组，暂不做）；
 *    · SG90（ID3 手腕 / ID4 爪子）= 模拟：需持续 50Hz 脉冲保持，停发脉冲
 *      = 真卸力（手掰无阻力矩）；
 *  - 四通道常态由硬件 50Hz 连续发波（写 compare 即持续），Pwm_Release 仅
 *    用于 SG90 卸力与全关断；
 *  - 两级安全链：bsp_pwm 上电 compare=0（无脉冲）→ 本模块 Init 发中位 +
 *    稳定窗（ROBOT_SERVO_CENTER_SETTLE_MS）内不可控。
 */
#ifndef F103RC_SERVO_H
#define F103RC_SERVO_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SERVO_ID1 = 0,      /* 大臂（数字，fail-hold）—— id↔关节以实机布线为准 */
    SERVO_ID2,          /* 小臂（数字） */
    SERVO_ID3,          /* 手腕（SG90） */
    SERVO_ID4,          /* 爪子（SG90） */
    SERVO_COUNT,
} ServoId_t;

/**
 * @brief  初始化全部四路：启动 PWM（幂等）+ 每 id 发中位脉冲
 * @retval true 恒成功（配置编译期定死，运行期无失败源）
 * @note   发中位脉冲后 ROBOT_SERVO_CENTER_SETTLE_MS 内 SetAngle 不生效
 *         （中位稳定窗，非阻塞 Bsp_GetMs 计时，上电猛冲防护）
 */
bool Servo_InitAll(void);

/**
 * @brief  设定目标角度
 * @param  id SERVO_ID1..4；非法值忽略
 * @param  deg 0..180°；NaN 忽略；按每 id 软限位钳位
 *         （ROBOT_SERVO_LIMIT_*_DEG_LIST，占位 ±5°）后按每 id 独立标定
 *         的脉宽两端（ROBOT_SERVO_PULSE_*_US_LIST）线性换算
 * @note   调用即恢复发脉冲（对已 Release 的 SG90 重新上力）
 */
void Servo_SetAngle(ServoId_t id, float deg);

/**
 * @brief  停发指定通道脉冲
 * @retval true = 本次停脉冲真卸力（SG90，手掰无保持力矩）；
 *         false = 数字舵机锁存不变（fail-hold，仍保持出力）或参数非法
 * @note   动作与结果一体返回（2026-10-03 变更：void→bool），调用即得知
 *         本次是否真卸力，日志/告警/上层状态语义一次拿到
 */
bool Servo_Release(ServoId_t id);

/**
 * @brief  能力查询：该关节软件能否卸力（W2 actuator 装配用，无副作用）
 * @retval true = SG90（停脉冲即卸力）；false = 数字舵机（停脉冲仍锁存，
 *         真卸力需硬件供电开关，暂无）
 * @note   2026-10-03 改名：ReleaseCutsPower → CanUnload——语义从"切断
 *         供电"的实现细节上移为"软件卸力能力"。不合并进 Release：
 *         装配期查询必须无副作用（手腕全程不能松，不允许为查询而真卸
 *         一次力），且"数字⇒停脉冲≠卸力"的器件解释权收在本模块一处
 */
bool Servo_CanUnload(ServoId_t id);

#endif /* F103RC_SERVO_H */
