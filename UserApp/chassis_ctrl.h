/**
 * @file    chassis_ctrl.h
 * @brief   底盘业务纯函数（UserApp）：遥控指令 → 四轮目标转速，零硬件依赖
 *
 * 职责边界（架构 §数据流 / Modules 规划 §五）：麦轮正逆解、手感曲线都属
 * "业务决策"，归 UserApp；本文件只做算术。本文件不 include 任何 bsp_xxx
 * 或 motor 头——调用方（chassis_task）拿输出喂 Motor_SetSpeedRpm。
 * 纯函数 + 调用方持有数据，PC 单测与数字孪生桥共用同一份实现。
 *
 * 三步流水（每步独立可测，孪生桥逐级透出中间量）：
 *   RC_Cmd 快照 → Chassis_Shape（死区+平方手感）
 *              → Chassis_ToPhysical（归一化 → m/s、rad/s）
 *              → Chassis_MecanumIK（物理速度 → 四轮 rpm）
 *
 * 输入适配纪律（2026-10-11 用户裁决）：遥控器上有什么物理输入（摇杆/按键/
 * 旋钮），由发送端折算成帧里的 vx/vy/omega 归一化量——车载只认字段，不读
 * keys 位图（keys 语义归发送端定义与消费）。数字孪生只是调试工具，其网页
 * 端键盘映射（main.js 折算进摇杆量）即此模式的参照，车载不为它做设计。
 *
 * 轮序约定：索引 0..3 = 左前FL/右前FR/左后RL/右后RR，与 MOTOR_CH1..4 的
 * 占位直连序一致；输出"正 rpm = 轮子向前滚"，电机电气方向修正归接线层
 * （ROBOT_MOTOR_SIGN / 通道映射表）。
 */
#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_config.h"

/* 底盘三自由度向量：视语境是"归一化比例"或"物理量 m/s、rad/s"（见函数注释） */
typedef struct
{
    float vx;       /* 前+（ROS REP-103：x 前） */
    float vy;       /* 左+（y 左） */
    float omega;    /* 逆时针+（z 上，左转为正） */
} Chassis_Vec_t;

#define CHASSIS_WHEEL_COUNT     4

/** 第一步：手感整形——死区（ROBOT_APP_CHASSIS_DEADBAND 内归零）后按平方
 *  曲线重映射（摇杆轻推细、推满猛），出力符号与输入一致、1.0 映射 1.0。
 *  曲线作用方式 = 向量整形（2026-10-20）：死区逐轴处理后，平方只捏行程最大
 *  的主轴，其余轴按主轴行程等比跟随。单轴输入与逐轴平方逐位一致；组合输入
 *  （单杆斜推的前进+回转）在整形后保持方向比例——交换率不被"小值平方更小"
 *  放大，斜推时前进按杆几何让位、旋转跟得上。
 *  输入 = rc_cmd 已归一化的比例量 -1.0..+1.0（帧整数 ÷满幅，归一化是协议
 *  解析的一部分，发生在 rc_cmd，发送端发整数即逻辑量本体、无需归一化）。
 *  急停/失联指令已被 rc_cmd 归零/渐停，此处照常整形即得安全输出。
 */
void Chassis_Shape(const Chassis_Vec_t *in, Chassis_Vec_t *out);

/** 第二步：归一化比例 → 物理速度（满幅 ROBOT_APP_VX/VY_MAX_MPS、W_MAX_RADPS） */
void Chassis_ToPhysical(const Chassis_Vec_t *norm, Chassis_Vec_t *phys);

/** 第三步：麦轮逆解（PROTOCOL.md §3 公式，γ=FL-1/FR+1/RL+1/RR-1 已含在式内）
 *  出四轮目标转速 rpm（正 = 轮子向前滚）。任一轮超出 ROBOT_MOTOR_SPEED_MAX_RPM
 *  时四轮等比缩放（保方向不保模长），不静默削峰单轮。
 */
void Chassis_MecanumIK(const Chassis_Vec_t *phys, float rpm_out[CHASSIS_WHEEL_COUNT]);

/** 麦轮正解：四轮角速度 rad/s（孪生 telemetry 的 wheel 量纲）→ 物理速度。
 *  与逆解互为逆运算，PC 单测做往返恒等；亦用于拿实车/孪生轮速反推车速。 */
void Chassis_MecanumFK(const float wheel_omega_radps[CHASSIS_WHEEL_COUNT],
                       Chassis_Vec_t *phys);

#endif /* CHASSIS_CTRL_H */
