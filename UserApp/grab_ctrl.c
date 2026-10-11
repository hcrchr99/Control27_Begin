/**
 * @file    grab_ctrl.c
 * @brief   抓取业务纯函数实现（设计说明见 grab_ctrl.h）
 */
#include "grab_ctrl.h"
#include <stddef.h>

/* 软限位（度）：直接借舵机限位占位表——同一个 0..180° 坐标系，实机机械
 * 限位定案后随 ROBOT_SERVO_LIMIT_* 一并收紧，本文件零改动 */
static const float kLimitMin[ROBOT_CMD_JOINT_COUNT] = ROBOT_SERVO_LIMIT_MIN_DEG_LIST;
static const float kLimitMax[ROBOT_CMD_JOINT_COUNT] = ROBOT_SERVO_LIMIT_MAX_DEG_LIST;

void Grab_Init(Grab_State_t *g, const float deg0[ROBOT_CMD_JOINT_COUNT])
{
    if (g == NULL)
    {
        return;
    }
    for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
    {
        float d = (deg0 != NULL) ? deg0[i] : 0.0f;
        if (d < kLimitMin[i])
        {
            d = kLimitMin[i];
        }
        if (d > kLimitMax[i])
        {
            d = kLimitMax[i];
        }
        g->deg[i] = d;
        g->rate[i] = 0.0f;
    }
    g->last_ms = 0u;
    g->started = false;
}

void Grab_Update(Grab_State_t *g, const float rate_cmd[ROBOT_CMD_JOINT_COUNT],
                 uint32_t now_ms)
{
    if (g == NULL)
    {
        return;
    }

    /* 首拍只对表不积分（上电第一拍 dt 未知，先记时基） */
    if (!g->started)
    {
        g->last_ms = now_ms;
        g->started = true;
        for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
        {
            g->rate[i] = 0.0f;
        }
        return;
    }

    /* uint32 无符号减法：时基回绕时 dt 依然正确（同 rc_cmd 手法） */
    uint32_t dt_ms = now_ms - g->last_ms;
    g->last_ms = now_ms;

    float step_s = (float)dt_ms * 0.001f * ROBOT_APP_JOINT_RATE_DPS;

    for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
    {
        float rate = (rate_cmd != NULL) ? rate_cmd[i] : 0.0f;
        g->rate[i] = rate;

        float d = g->deg[i] + rate * step_s;

        /* 软限位钳停：顶到头就停在那。⚠ 与 Modules/servo 的钳位不是重复：
         * 本钳位是防积分逸出（不钳则 deg 在限位外持续累积，反向时先走完
         * 逸出量才有动作=反向死区）；servo 钳位是器件契约（任何调用方
         * 兜底）。两层必须，且同引 ROBOT_SERVO_LIMIT_*_DEG_LIST 单一来源 */
        if (d < kLimitMin[i])
        {
            d = kLimitMin[i];
        }
        if (d > kLimitMax[i])
        {
            d = kLimitMax[i];
        }
        g->deg[i] = d;
    }
}
