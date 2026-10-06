/**
 * @file    servo.h
 * @brief   舵机 ×5（TIM9 250Hz PM10S / TIM5 50Hz SG90 双组脉宽）：脉宽↔角度、
 *          软限位、卸力（S5 器件层；2026-10-05 随 F407VG 迁移增补 ID5=爪旋转）
 *
 * 分层与边界（Modules 层开发规划 §3.2）：
 *  - 本模块只回答"怎么动"（器件知识：脉宽↔角度、软限位、卸力），不知道
 *    "何时动、动多久"——那是 actuator（W2.4）包装的状态机，再上是 grab；
 *  - 器件分型（robot_config.h ROBOT_SERVO_*_LIST 配置）：
 *    · PM10S 数字（ID1 大臂 / ID2 小臂，250Hz 组）
 *    · SG90（ID3 手腕 / ID4 爪开合 / ID5 爪旋转，50Hz 组）
 *    ⚠ 卸力语义（2026-10-05 实测修正）：停脉冲=卸力对两类一致，PM10S
 *    并非 fail-hold（此前"停脉冲≠卸力、需硬件 MOS 开关"的表述作废）。
 *    分型差异只剩帧率组与 W2.4 actuator 的判据选择（电流 vs 热积累），
 *    不再关联卸力能力；
 *  - 五通道常态由硬件按组连续发波（写 compare 即持续），Pwm_Release 停
 *    脉冲即卸力，SetAngle 自动恢复发脉冲；
 *  - 两级安全链：bsp_pwm 上电 compare=0（无脉冲）→ 本模块 Init 发中位 +
 *    稳定窗（ROBOT_SERVO_CENTER_SETTLE_MS）内不可控。
 */
#ifndef SERVO_H
#define SERVO_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    SERVO_ID1 = 0,      /* 大臂（PM10S）—— id↔关节以实机布线为准 */
    SERVO_ID2,          /* 小臂（PM10S） */
    SERVO_ID3,          /* 手腕（SG90） */
    SERVO_ID4,          /* 爪开合（SG90） */
    SERVO_ID5,          /* 爪旋转（SG90，2026-10-05 新增第 5 路） */
    SERVO_COUNT,
} ServoId_t;

/**
 * @brief  初始化全部五路：启动 PWM（幂等）+ 每 id 发中位脉冲
 * @retval true 恒成功（配置编译期定死，运行期无失败源）
 * @note   发中位脉冲后 ROBOT_SERVO_CENTER_SETTLE_MS 内 SetAngle 不生效
 *         （中位稳定窗，非阻塞 Bsp_GetMs 计时，上电猛冲防护）
 */
bool Servo_InitAll(void);

/**
 * @brief  设定目标角度
 * @param  id SERVO_ID1..5；非法值忽略
 * @param  deg 0..180°；NaN 忽略；按每 id 软限位钳位
 *         （ROBOT_SERVO_LIMIT_*_DEG_LIST，占位 ±5°）后按每 id 独立标定
 *         的脉宽两端（ROBOT_SERVO_PULSE_*_US_LIST）线性换算
 * @note   调用即恢复发脉冲（对已 Release 的通道重新上力）
 */
void Servo_SetAngle(ServoId_t id, float deg);

/**
 * @brief  停发指定通道脉冲 = 卸力（PM10S / SG90 一致）
 * @retval true = 已停脉冲卸力；false = 参数非法
 * @note   动作与结果一体返回，调用即得知；SetAngle 自动重新上力
 */
bool Servo_Release(ServoId_t id);

#endif /* SERVO_H */
