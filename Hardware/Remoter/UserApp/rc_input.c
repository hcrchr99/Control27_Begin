/**
 * @file    rc_input.c
 * @brief   发送端输入适配实现（纯函数，见 rc_input.h）
 */
#include "rc_input.h"
#include <string.h>

/* 单轴归一化：去死区 → 按物理端点线性拉满 ±ROBOT_CMD_RANGE → 钳位。
 * 正/负半程分开算跨度（中点一般不在 2048 正中，两端物理行程不等长）；
 * 整数运算，结果可精确复现（PcTests 断言用同式手算值） */
static int16_t Axis_Normalize(uint16_t raw, uint16_t center, uint16_t deadband)
{
    int32_t d = (int32_t)raw - (int32_t)center;

    if (d > (int32_t)deadband)
    {
        d -= (int32_t)deadband;
    }
    else if (d < -(int32_t)deadband)
    {
        d += (int32_t)deadband;
    }
    else
    {
        return 0;
    }

    if (d > 0)
    {
        int32_t span = (int32_t)(4095u - center - deadband);
        if (span <= 0)
        {
            return 0;
        }
        d = d * (int32_t)ROBOT_CMD_RANGE / span;
        if (d > (int32_t)ROBOT_CMD_RANGE)
        {
            d = (int32_t)ROBOT_CMD_RANGE;
        }
    }
    else
    {
        int32_t span = (int32_t)(center - deadband);
        if (span <= 0)
        {
            return 0;
        }
        d = d * (int32_t)ROBOT_CMD_RANGE / span;
        if (d < -(int32_t)ROBOT_CMD_RANGE)
        {
            d = -(int32_t)ROBOT_CMD_RANGE;
        }
    }
    return (int16_t)d;
}

void RcInput_Build(const RcInput_Config_t *cfg, const RcInput_Raw_t *raw,
                   uint8_t seq, RCPayload_t *out)
{
    if (cfg == NULL || raw == NULL || out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof *out);    /* 发送端纪律：整帧清零再填 */

    out->vx = Axis_Normalize(raw->joy_x, cfg->joy_center, cfg->joy_deadband);
    out->vy = Axis_Normalize(raw->joy_y, cfg->joy_center, cfg->joy_deadband);
    /* omega 恒 0：旋转轴输入待拍板（见 rc_input.h） */

    for (int k = 0; k < ROBOT_CMD_JOINT_COUNT; k++)
    {
        out->joint[k] = ((raw->keys & (uint8_t)(1u << k)) != 0u)
                        ? cfg->key_joint_rate : 0;
    }

    out->keys = raw->keys;              /* 位图透传，含义归 UserApp/车端 */
    if (raw->estop != 0u)
    {
        out->flags |= RC_FLAG_ESTOP;    /* 车端闩锁、松键帧解除 */
    }
    out->seq = seq;
}
