/**
 * @file    actuator.c
 * @brief   执行器保护层实现：状态机 + 堵转确认窗 + 1s 粒度滑动热累计
 *          （接口与状态机文档见 actuator.h）
 *
 * 实现要点（规划 §3.3）：
 *  - 判据/累计器全在 Act_Update 节拍内闭环，不经 daemon；
 *  - 电压式滑桶：30×1s 环形桶，按真实时间滑动（与是否供电无关），
 *    供电才往当前桶累加——停 5s 恢复供电时旧桶已滑出，窗口不失真；
 *  - 所有跨时刻比较用无符号差分（u32 回绕自动正确）；时间一律调用方
 *    注入，本模块零硬件依赖（PC 单测直接手推时戳）。
 */
#include "actuator.h"
#include "ease.h"
#include <string.h>

bool Act_Init(Actuator_t *a, const ActConfig_t *cfg)
{
    if (a == NULL || cfg == NULL)
    {
        return false;
    }
    if ((uint32_t)cfg->servo_id >= (uint32_t)SERVO_COUNT)
    {
        return false;
    }
    if (!(cfg->ease_dps > 0.0f))        /* NaN 亦落入此判 */
    {
        return false;
    }
    if (!(cfg->min_deg < cfg->max_deg))
    {
        return false;
    }

    a->cfg = *cfg;

    uint32_t n = cfg->energize_window_ms / 1000u;
    if (n > ACT_BUCKET_MAX)
    {
        n = ACT_BUCKET_MAX;             /* 超长窗口按 30s 生效（向更严侧收） */
    }
    memset(a->bucket, 0, sizeof(a->bucket));
    a->n_bucket = (uint8_t)n;
    a->bucket_head = 0u;
    a->bucket_slide_ms = 0u;        /* 首拍防御分支统一吸收（见 Update） */

    a->state = ACT_IDLE;
    a->cur_deg = 90.0f;             /* servo InitAll 发的中位脉宽对应 90° */
    a->target_deg = 90.0f;
    a->timed = false;
    a->t_move_ms = 0u;
    a->move_start_ms = 0u;
    a->over_th = false;
    a->over_start_ms = 0u;
    a->stalled = false;
    a->hot = false;
    a->idle_t0_valid = false;
    a->idle_t0_ms = 0u;

    a->inited = true;
    return true;
}

void Act_SetTarget(Actuator_t *a, float deg)
{
    if (a == NULL || !a->inited || deg != deg)
    {
        return;
    }
    if (deg < a->cfg.min_deg)
    {
        deg = a->cfg.min_deg;
    }
    if (deg > a->cfg.max_deg)
    {
        deg = a->cfg.max_deg;
    }

    a->target_deg = deg;
    a->stalled = false;             /* 新动作 = 堵转锁存清除点，重新起判 */
    a->over_th = false;

    float dist = deg - a->cur_deg;
    if (dist < 0.0f)
    {
        dist = -dist;
    }
    uint32_t t_move = (uint32_t)(dist / a->cfg.ease_dps * 1000.0f + 0.5f);
    if (t_move == 0u)
    {
        t_move = 1u;                /* 原地目标也走一拍 MOVING，时序统一 */
    }
    a->t_move_ms = t_move;
    a->move_start_ms = a->last_ms;  /* 基准 = 最近 Update；未对表则首拍重锚 */
    a->state = ACT_MOVING;
}

void Act_Update(Actuator_t *a, uint32_t now_ms)
{
    if (a == NULL || !a->inited)
    {
        return;
    }

    if (!a->timed)
    {   /* 首拍：只对表（SetTarget 早于首拍时斜坡起点在此重锚，
         * 防止"巨大首拍 dt 一步瞬移到目标"） */
        a->last_ms = now_ms;
        a->timed = true;
        if (a->state == ACT_MOVING)
        {
            a->move_start_ms = now_ms;
        }
        return;
    }
    uint32_t dt = now_ms - a->last_ms;  /* 无符号差分，回绕自动正确 */
    a->last_ms = now_ms;

    /* ---- 状态推进 + 发脉冲 ---- */
    switch (a->state)
    {
    case ACT_MOVING:
        if ((now_ms - a->move_start_ms) >= a->t_move_ms)
        {
            a->cur_deg = a->target_deg;     /* 到点精确落位（避免步进残差） */
            a->state = ACT_HOLDING;
        }
        else
        {
            float step = a->cfg.ease_dps * ((float)dt * 0.001f);
            a->cur_deg = Ease_Step(a->cur_deg, a->target_deg, step);
        }
        Servo_SetAngle(a->cfg.servo_id, a->cur_deg);
        break;

    case ACT_HOLDING:
        Servo_SetAngle(a->cfg.servo_id, a->target_deg); /* 恒发（规划 §3.3） */
        break;

    default:    /* IDLE / RELEASED 不发角度 */
        break;
    }

    bool powered = (a->state == ACT_MOVING) || (a->state == ACT_HOLDING);

    /* ---- 堵转判据：超阈值持续确认窗才坐实（瞬时冲击不误触发）；
     * NaN 电流参与比较恒假 → 采样通道读不到时绝不误触发 ---- */
    if (powered && (a->cfg.get_current_a != NULL) && (a->cfg.stall_current_a > 0.0f))
    {
        float i = a->cfg.get_current_a();
        if (i >= a->cfg.stall_current_a)
        {
            if (!a->over_th)
            {
                a->over_th = true;
                a->over_start_ms = now_ms;
            }
            else if ((now_ms - a->over_start_ms) >= a->cfg.stall_confirm_ms)
            {
                a->stalled = true;
                Act_Release(a);     /* 判据触发卸力；何时允许再动由业务拍板 */
            }
        }
        else
        {
            a->over_th = false;
        }
    }
    else
    {
        a->over_th = false;
    }

    /* ---- 热保护滑动累计（仅使能实例；只报告不强制卸力） ---- */
    if ((a->cfg.energize_max_ms > 0u) && (a->n_bucket > 0u))
    {
        if ((now_ms - a->bucket_slide_ms) >= (uint32_t)a->n_bucket * 1000u)
        {   /* 整窗都已过去（长眠恢复/初值 0）：直接全清重新计 */
            memset(a->bucket, 0, sizeof(a->bucket));
            a->bucket_head = 0u;
            a->bucket_slide_ms = now_ms;
        }
        else
        {
            while ((now_ms - a->bucket_slide_ms) >= 1000u)
            {
                a->bucket_head = (uint8_t)((a->bucket_head + 1u) % a->n_bucket);
                a->bucket[a->bucket_head] = 0u;
                a->bucket_slide_ms += 1000u;
            }
        }

        if (powered)
        {
            uint32_t v = (uint32_t)a->bucket[a->bucket_head] + dt;
            a->bucket[a->bucket_head] = (v > 1000u) ? 1000u : (uint16_t)v;
        }

        uint32_t sum = 0u;
        for (uint8_t k = 0u; k < a->n_bucket; k++)
        {
            sum += a->bucket[k];
        }
        if (!a->hot && (sum >= a->cfg.energize_max_ms))
        {
            a->hot = true;      /* 只置位：拍板（何时卸力）在业务层 */
        }

        if (a->hot)
        {
            if (!powered)
            {
                if (!a->idle_t0_valid)
                {
                    a->idle_t0_valid = true;
                    a->idle_t0_ms = now_ms;
                }
                else if ((now_ms - a->idle_t0_ms) >= a->cfg.cooldown_ms)
                {   /* 冷却够久：解除并清零重计 */
                    a->hot = false;
                    a->idle_t0_valid = false;
                    memset(a->bucket, 0, sizeof(a->bucket));
                }
            }
            else
            {
                a->idle_t0_valid = false;   /* 供电打断冷却计时 */
            }
        }
    }
}

void Act_Release(Actuator_t *a)
{
    if (a == NULL || !a->inited)
    {
        return;
    }
    (void)Servo_Release(a->cfg.servo_id);   /* 停脉冲=卸力（两类器件一致） */
    a->state = ACT_RELEASED;
    a->over_th = false;     /* 判定随供电停止复位；stalled 锁存保留到下个 SetTarget */
}

bool Act_IsSettled(const Actuator_t *a)
{
    return (a != NULL) && a->inited && (a->state == ACT_HOLDING);
}

bool Act_IsStalled(const Actuator_t *a)
{
    return (a != NULL) && a->inited && a->stalled;
}

bool Act_NeedsCooldown(const Actuator_t *a)
{
    return (a != NULL) && a->inited && a->hot;
}
