/**
 * @file    rc_input.h
 * @brief   发送端输入适配：船型开关 + 摇杆原始读数 → RC_Cmd v2 协议帧（纯函数）
 *
 * 职责边界（输入适配归发送端，2026-10-11 定案）：发送端只把物理输入整理成
 * 逻辑量（设备清噪死区、按 键名→帧字段 映射填帧）；满量程/手感/物理换算
 * 归车端，本模块不做。整帧清零再填（发送端纪律）也在此收口。
 *
 * 帧字段语义（rc_cmd.h v2 冻结布局）：
 *   vx/vy      = 摇杆 X/Y（设备死区滤除后线性归一化到 ±ROBOT_CMD_RANGE）
 *   omega      = 恒 0（旋转轴输入待拍板：候选=摇杆按压键临时切轴/另配旋钮）
 *   joint[k]   = K_(k+1) 按住 → +key_joint_rate，松开 → 0（反向语义待拍板）
 *   keys       = 开关位图透传（bit0..4=K_1..K_5，bit5=K_EB；含义归 UserApp）
 *   flags      = bit0 estop（K_EB 按下置 1；车端闩锁、松键帧解除）
 *   seq        = 调用方递增透传（只在本帧实发时 ++，与 PTX 纪律一致）
 *
 * 纯函数无硬件依赖：只 include rc_cmd.h，配置经参数注入（PcTests 友好；
 * 固件侧默认值由 remoter_app.c 用 robot_config.h 的宏组装）。
 */
#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <stdint.h>
#include "rc_cmd.h"

/* 输入适配配置（遥控器标定参数集中注入） */
typedef struct
{
    uint16_t joy_center;    /* 摇杆松手中点（raw counts，实测标定） */
    uint16_t joy_deadband;  /* 设备清噪死区（counts）：|raw-center|≤死区输出 0 */
    int16_t  key_joint_rate;/* 开关按住的关节速率（±ROBOT_CMD_JOINT_RANGE 内） */
} RcInput_Config_t;

/* 一拍物理输入快照（采集侧填，RcInput_Build 消费） */
typedef struct
{
    uint16_t joy_x;         /* 摇杆 X 原始值 0..4095 */
    uint16_t joy_y;         /* 摇杆 Y 原始值 0..4095 */
    uint8_t  keys;          /* bit0..4 = K_1..K_5，1=按下（船型开关=保持态） */
    uint8_t  estop;         /* 1 = K_EB 按下（急停） */
} RcInput_Raw_t;

/**
 * @brief  由输入快照构建一帧协议数据（整帧清零再填，"发送端纪律"唯一收口点）
 * @param  cfg   标定配置（中点/死区/关节速率）
 * @param  raw   本拍输入快照
 * @param  seq   发送序号（调用方维护，本函数透传不自增）
 * @param  out   输出帧，不可为 NULL
 */
void RcInput_Build(const RcInput_Config_t *cfg, const RcInput_Raw_t *raw,
                   uint8_t seq, RCPayload_t *out);

#endif /* RC_INPUT_H */
