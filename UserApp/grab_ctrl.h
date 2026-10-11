/**
 * @file    grab_ctrl.h
 * @brief   抓取业务纯函数（UserApp）：关节速率指令 → 五关节目标角，零硬件依赖
 *
 * 职责边界：只做"速率积分 + 软限位"的算术，产出五路目标角（舵机 0..180°
 * 坐标，索引序 = joint[0..4] = 舵机 ID1..5 = 大臂/小臂/手腕/爪开合/爪旋转）；
 * 不 include servo/actuator——拿目标角喂执行器是 grab_task 的事。
 *
 * 输入适配纪律（2026-10-11 用户裁决，同 chassis_ctrl）：遥控器上的关节点动
 * 键若将来存在，由发送端折算成帧里的 joint[i] 速率字段；车载不读 keys 位图
 * （keys 语义归发送端定义与消费）。调用方（grab_task）从 RC_Cmd 快照摘出
 * joint[5] 传入——本模块不认识协议类型（同 chassis 吃 Chassis_Vec_t 的口径）。
 *
 * 安全语义对齐 rc_cmd（两级故障，本模块不再另造）：传入的 rate_cmd 已内含
 * 全部语义——急停被 rc_cmd 归零（速率零 = 角度保持，不回零，矿不松爪）、
 * 失联 300ms 渐停（照常积分缩小的速率即得渐停位移），本模块直接积分即可。
 *
 * 爪开合（joint[3]）在本模块同为 0..180° 舵机角；孪生侧是 0..12mm 平移行程，
 * 两侧映射各自维护，桥接对表时以本模块角度为准。
 */
#ifndef GRAB_CTRL_H
#define GRAB_CTRL_H

#include <stdbool.h>
#include <stdint.h>
#include "robot_config.h"

/* 速率积分状态（调用方持有实例，本模块零静态状态） */
typedef struct
{
    float deg[ROBOT_CMD_JOINT_COUNT];   /* 目标角（舵机 0..180° 坐标，已限位） */
    float rate[ROBOT_CMD_JOINT_COUNT];  /* 本拍取用速率 -1..+1（诊断/孪生下行可见） */
    uint32_t last_ms;                   /* 上拍时基（dt 来源） */
    bool started;                       /* 首拍已对表（首拍 dt=0 只对表不积分） */
} Grab_State_t;

/** 清态：目标角置 deg0（越限部分钳进软限位），速率清零，等首拍对表 */
void Grab_Init(Grab_State_t *g, const float deg0[ROBOT_CMD_JOINT_COUNT]);

/** 节拍推进：吃一拍关节速率指令，积分出本拍目标角与取用速率
 *  @param rate_cmd 本拍五关节归一化速率 -1..+1（调用方从 RC_Cmd 快照摘出；
 *                  NULL 按全零处理=角度保持）
 *  @param now_ms   当前毫秒时基（调用方递入；uint32 回绕安全，内部无符号减法）
 */
void Grab_Update(Grab_State_t *g, const float rate_cmd[ROBOT_CMD_JOINT_COUNT],
                 uint32_t now_ms);

#endif /* GRAB_CTRL_H */
