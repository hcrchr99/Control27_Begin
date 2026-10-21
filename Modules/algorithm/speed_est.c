/**
 * @file    speed_est.c
 * @brief   M/T 测速实现（纯算法，见 speed_est.h）
 */
#include "speed_est.h"
#include <stddef.h>

void SpeedEst_Init(SpeedEst_t *se, const SpeedEst_Config_t *cfg)
{
    if (se == NULL || cfg == NULL)
    {
        return;
    }
    se->cfg = *cfg;
    if (se->cfg.ppr < 1.0f)
    {
        se->cfg.ppr = 1.0f;     /* 分母兜底：配置漏填也不出 INF */
    }
    se->newest.cnt = 0;
    se->newest.us = 0u;
    se->have_newest = false;
    se->out = 0.0f;
    se->primed = false;
    se->last_upd_us = 0u;
    se->dbg_stale_age_us = 0u;
    se->dbg_stale_len = 0u;
    se->dbg_stale_n = 0u;
    se->dbg_stale_cnt = 0u;
}

float SpeedEst_Update(SpeedEst_t *se, const SpeedEst_Sample_t *samples,
                      uint32_t n, uint32_t now_us)
{
    if (se == NULL)
    {
        return 0.0f;
    }

    /* 本拍关心的"最新样本"：有新喂的取批尾，没新喂的用上次记下的
     * （两个边沿之间编码器没有输出任何信息，保持旧读数就是最诚实的估计） */
    const SpeedEst_Sample_t *newest = NULL;
    bool fed = false;
    if (n > 0u && samples != NULL)
    {
        newest = &samples[n - 1u];
        fed = true;
    }
    else if (se->have_newest)
    {
        newest = &se->newest;
    }
    else
    {
        return se->out;         /* 上电以来一个样本都没见过：维持初值 0 */
    }

    /* 静止判停：最新样本已经老过 stale_us，说明边沿稀到没有了信息，归 0。
     * 顺手清低通的首值标记——恢复转动后的第一个读数直通，不从 0 慢慢爬。
     * ⚠ 必须有符号比较：样本时间戳出自边沿中断、now_us 出自本任务拍首，
     * 拍内被中断抢占会让最新样本"比 now 快 1µs"——无符号差分瞬间变成
     * 71.6 分钟超龄（age=0xFFFFFFFF），误判停归零，边沿越密踩中越频
     * （S-C 板验满屏扎零的真凶，2026-10-21 病灶记录仪实锤 age=4294967ms） */
    if ((int32_t)(now_us - newest->us) > (int32_t)se->cfg.stale_us)
    {
        se->dbg_stale_age_us = (uint32_t)(now_us - newest->us);
        se->dbg_stale_len = (n > 0u && samples != NULL) ? n : 0u;
        se->dbg_stale_n = se->dbg_stale_len;
        se->dbg_stale_cnt++;
        se->out = 0.0f;
        se->primed = false;
        se->last_upd_us = now_us;
        if (fed)
        {
            se->newest = *newest;
            se->have_newest = true;
        }
        return 0.0f;
    }

    /* 找窗起点：从批尾往批头扫，要"比最新样本老 ≥win_min"的样本里最新的
     * 那个——窗越短越新鲜，够到 win_min 就收。本批跨度不够（低转速下一拍
     * 常常只有 0~1 个边沿）就退回上一批的最新样本搭跨批窗。
     * ⚠ 起点必须按值拷走：跨批起点就是 se->newest 本尊，下面要往里写新
     * 样本，按指针引用会被当场覆盖成最新样本（Δt 恒 0、永远走保持分支） */
    SpeedEst_Sample_t start;
    bool have_start = false;
    if (n > 1u)
    {
        for (uint32_t i = n - 1u; i-- > 0u; )
        {
            if ((newest->us - samples[i].us) >= se->cfg.win_min_us &&
                samples[i].us != newest->us)
            {
                start = samples[i];
                have_start = true;
                break;
            }
        }
    }
    if (!have_start && se->have_newest)
    {
        uint32_t cross_age = newest->us - se->newest.us;
        /* 跨批借"上一批最新样本"搭窗：只借新鲜度地平线（stale）以内的——
         * 停喂/掉队恢复的第一拍，这个老家伙跨着断档，拿它搭窗就是编数据 */
        if (cross_age >= se->cfg.win_min_us &&
            cross_age <= se->cfg.stale_us &&
            se->newest.us != newest->us)
        {
            start = se->newest;
            have_start = true;
        }
    }
    if (fed)
    {
        se->newest = *newest;
        se->have_newest = true;
    }
    if (!have_start)
    {
        return se->out;         /* 窗还攒不到 win_min：维持上次读数 */
    }

    /* M/T 核心式：窗两端都吸附在真实边沿上，Δcnt 是精确整数、Δt 是精确
     * 微秒，分数计数一个不丢——量化误差只剩时间戳的 µs 级噪声。
     * uint32 相减跨回绕天然正确；Δt=0（同拍重复时间戳）走保持分支防除零 */
    uint32_t dt_us = newest->us - start.us;
    if (dt_us == 0u)
    {
        return se->out;
    }
    float raw = 60.0f * (float)(newest->cnt - start.cnt) /
                (se->cfg.ppr * ((float)dt_us * 1e-6f));

    /* 输出低通：一阶 RC，步长按本次 Update 的真实间隔算（rc=0 完全透传）。
     * 只在算出新读数时滤，保持直通的旧值不再复滤——滤的是新信息，不是旧账 */
    float out = raw;
    if (se->cfg.lpf_rc_us > 0.0f)
    {
        uint32_t dt_upd = now_us - se->last_upd_us;
        if (!se->primed || dt_upd == 0u)
        {
            se->primed = true;  /* 首值直通，不从 0 爬坡 */
        }
        else
        {
            float a = (float)dt_upd / (se->cfg.lpf_rc_us + (float)dt_upd);
            out = se->out + (raw - se->out) * a;
        }
    }
    else
    {
        se->primed = true;
    }
    se->last_upd_us = now_us;
    se->out = out;
    return out;
}

/* ======================= 滴喂包装（见 speed_est.h） ======================= */

void SpeedEstDrip_Init(SpeedEstDrip_t *dp)
{
    if (dp == NULL)
    {
        return;
    }
    dp->len = 0u;
}

float SpeedEstDrip_Update(SpeedEstDrip_t *dp, SpeedEst_t *se,
                          const SpeedEst_Sample_t *samples, uint32_t n,
                          uint32_t now_us)
{
    if (dp == NULL || se == NULL)
    {
        return 0.0f;
    }

    /* 本拍新样本进历史；满深先裁一段历史再落尾（正常按窗龄裁用不到这兜底） */
    for (uint32_t j = 0u; j < n; j++)
    {
        if (dp->len < SPEED_EST_DRIP_DEPTH)
        {
            dp->hist[dp->len] = samples[j];
            dp->len++;
        }
        else
        {
            for (uint16_t k = 1u; k < dp->len; k++)
            {
                dp->hist[k - 1u] = dp->hist[k];
            }
            dp->hist[dp->len - 1u] = samples[j];
        }
    }

    /* 按窗龄裁历史：跨度压在 2×win_min 内。作用有两个——历史不无限变老；
     * 停喂恢复的第一拍把跨停喂段的旧样本裁掉，窗在几毫秒内重新攒齐，
     * 不会拿"跨停喂 40ms 的 ΔN/Δt"编出一个假转速 */
    if (dp->len > 1u)
    {
        uint32_t span_cap = se->cfg.win_min_us * 2u;
        while (dp->len > 1u &&
               (dp->hist[dp->len - 1u].us - dp->hist[1u].us) > span_cap)
        {
            for (uint16_t k = 1u; k < dp->len; k++)
            {
                dp->hist[k - 1u] = dp->hist[k];
            }
            dp->len--;
        }
    }

    return SpeedEst_Update(se, dp->hist, dp->len, now_us);
}
