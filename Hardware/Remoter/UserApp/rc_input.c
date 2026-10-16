/**
 * @file    rc_input.c
 * @brief   发送端输入适配实现（纯函数，见 rc_input.h）
 */
#include "rc_input.h"
#include <string.h>

/* 单轴适配：去死区 → 按物理端点线性拉满 ±ROBOT_CMD_RANGE → 钳位。
 * 正/负半程分开算跨度（中点一般不在 2048 正中，两端物理行程不等长）；
 * 整数运算，结果可精确复现（PcTests 断言用同式手算值） */
int16_t RcInput_NormalizeAxis(uint16_t raw, uint16_t center, uint16_t deadband)
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

uint16_t RcInput_Debounce(uint16_t debounced, uint16_t prev_raw, uint16_t raw_now)
{
    /* 两拍同为 1 → 翻 1；两拍同为 0 → 清 0；前后不一致（弹跳中）→ 维持原态。
     * 位运算一次处理整个字，各位独立互不牵连 */
    uint16_t settled = (uint16_t)(raw_now & prev_raw);
    uint16_t cleared = (uint16_t)((uint16_t)(~raw_now) & (uint16_t)(~prev_raw));

    return (uint16_t)((debounced & (uint16_t)~(settled | cleared)) | settled);
}

void RcInput_DclickReset(RcInput_DclickState_t *st)
{
    if (st != NULL)
    {
        memset(st, 0, sizeof *st);
    }
}

uint8_t RcInput_DclickStep(RcInput_DclickState_t *st, uint8_t level,
                           uint32_t now_ms, const RcInput_DclickCfg_t *cfg)
{
    if (st == NULL || cfg == NULL)
    {
        return 0;
    }

    /* 1) 按下沿与超时逻辑（可能置保持计数） */
    if (level != 0u && st->prev_level == 0u)
    {
        if (st->armed != 0u &&
            (now_ms - st->first_press_ms) <= (uint32_t)cfg->window_ms)
        {
            st->hold_left = cfg->hold_ticks;    /* 双击成立 */
            st->armed = 0u;
        }
        else
        {
            st->armed = 1u;                     /* 记作第一按 */
            st->first_press_ms = now_ms;
        }
    }
    else if (level == 0u && st->armed != 0u &&
             (now_ms - st->first_press_ms) > (uint32_t)cfg->window_ms)
    {
        st->armed = 0u;                         /* 第一按超时，放弃（单击） */
    }

    /* 2) 事件保持输出：沿当拍即起，共 hold_ticks 拍 */
    uint8_t event = 0u;
    if (st->hold_left > 0u)
    {
        event = 1u;
        st->hold_left--;
    }

    st->prev_level = level;
    return event;
}

void RcInput_Build(const RcInput_Config_t *cfg, const RcInput_Raw_t *raw,
                   uint8_t seq, RCPayload_t *out)
{
    if (cfg == NULL || raw == NULL || out == NULL)
    {
        return;
    }

    memset(out, 0, sizeof *out);    /* 发送端纪律：整帧清零再填 */

    /* 轴逻辑量直通+钳位（发送端纪律：rc_cmd.h v2 vx/vy 由发送端钳位）；
     * PHYS 的去死区/归一化已在采集侧 RcInput_NormalizeAxis 完成，
     * WEB 的逻辑量由网页直传，两条路在这里汇合后语义一致 */
    out->vx = (raw->vx > ROBOT_CMD_RANGE) ? ROBOT_CMD_RANGE
            : ((raw->vx < -ROBOT_CMD_RANGE) ? -ROBOT_CMD_RANGE : raw->vx);
    out->vy = (raw->vy > ROBOT_CMD_RANGE) ? ROBOT_CMD_RANGE
            : ((raw->vy < -ROBOT_CMD_RANGE) ? -ROBOT_CMD_RANGE : raw->vy);

    /* K_6 → 整车旋转：A/B 刀同关节矛盾语义，失安全归零（2026-10-16 拍板） */
    {
        uint16_t a = (uint16_t)(raw->keys & RCINPUT_KEY_K6_A);
        uint16_t b = (uint16_t)(raw->keys & RCINPUT_KEY_K6_B);
        if ((a != 0u) && (b == 0u))
        {
            out->omega = cfg->key_omega_rate;
        }
        else if ((b != 0u) && (a == 0u))
        {
            out->omega = (int16_t)(-cfg->key_omega_rate);
        }
    }

    /* 方向刀解码：K_n 的 A/B 刀分别映射 joint[n-1] 正/反向（自复位中位=全 0）；
     * 同开关 A/B 同 1 = 矛盾输入（物理上不应出现），按失安全处理为 0 */
    for (int k = 0; k < ROBOT_CMD_JOINT_COUNT; k++)
    {
        uint16_t a = (uint16_t)(raw->keys & (uint16_t)(RCINPUT_KEY_K1_A << (2u * k)));
        uint16_t b = (uint16_t)(raw->keys & (uint16_t)(RCINPUT_KEY_K1_B << (2u * k)));
        int16_t  cmd = 0;

        if ((a != 0u) && (b == 0u))
        {
            cmd = cfg->key_joint_rate;
        }
        else if ((b != 0u) && (a == 0u))
        {
            cmd = (int16_t)(-cfg->key_joint_rate);
        }
        out->joint[k] = cmd;
    }

    out->keys = raw->keys;              /* 位图透传，含义归 UserApp/车端 */
    if (raw->estop != 0u)
    {
        out->flags |= RC_FLAG_ESTOP;    /* 车端闩锁、松键帧解除 */
    }
    out->seq = seq;
}
