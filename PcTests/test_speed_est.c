/**
 * @file    test_speed_est.c
 * @brief   M/T 测速纯逻辑单测（mt-speed-pending S-A）
 *
 * 覆盖点：恒速重构零量化、低速稀边沿+空拍保持、静止判停与恢复直通、
 * 窗内方向反转的符号、µs 时间戳跨回绕、输出低通透传/RC 步进、退化输入
 * 不出 NaN、win_min=0 退化两样本。全部注入合成样本流，不碰任何硬件桩。
 */
#include <math.h>
#include <stdint.h>
#include "speed_est.h"
#include "test_util.h"

/* 本文件的"真值口径"：60 × Δcnt ÷ (2496 × Δt_s)，与被测模块同一公式——
 * 测试的目的不是验证公式本身，而是验证窗口吸附/保持/判停/回绕这些
 * "公式外面的逻辑"不会把真值带歪 */
#define PPR     2496.0f
#define RAW(dc, us)  (60.0f * (float)(dc) / (PPR * ((float)(us) * 1e-6f)))

/* 用例 1：恒速重构零量化。30rpm 上下（每边沿 2 计数、沿距 1603µs）的
 * 合成流一次喂入，读数须贴住真值——旧 M 法 20ms 窗在这个转速只能给出
 * 1.2rpm 一格的台阶，这里用 1e-3 的紧 eps 证明台阶没了 */
static void TestConst_NoQuantization(void)
{
    SpeedEst_t se;
    SpeedEst_Config_t c = { PPR, 4000u, 30000u, 0.0f };
    SpeedEst_Init(&se, &c);

    SpeedEst_Sample_t s[128];
    for (uint32_t i = 0u; i < 128u; i++)
    {
        s[i].cnt = (int32_t)(i + 1u) * 2;
        s[i].us = (i + 1u) * 1603u;
    }
    float r = SpeedEst_Update(&se, s, 128u, s[127].us);
    CHECK_NEAR(r, RAW(2, 1603.0f), 1e-3f);

    /* 下一拍没有新边沿：保持读数（两个边沿之间没有信息，保持不是掉 0） */
    r = SpeedEst_Update(&se, NULL, 0u, s[127].us + 1000u);
    CHECK_NEAR(r, RAW(2, 1603.0f), 1e-3f);
}

/* 用例 2：低速稀边沿。11rpm（沿距 4371µs > win_min 4000µs）下跨批窗和
 * 空拍保持——这是 1kHz 消费者在真实低速下的主要路径 */
static void TestLowSpeed_SparseHold(void)
{
    SpeedEst_t se;
    SpeedEst_Config_t c = { PPR, 4000u, 30000u, 0.0f };
    SpeedEst_Init(&se, &c);

    SpeedEst_Sample_t s[3];
    for (uint32_t i = 0u; i < 3u; i++)
    {
        s[i].cnt = (int32_t)(i + 1u) * 2;
        s[i].us = (i + 1u) * 4371u;
    }

    /* 单边沿小批：窗攒不够，保持初值 0 */
    float r = SpeedEst_Update(&se, &s[0], 1u, s[0].us);
    CHECK(r == 0.0f);

    /* 第二拍来一个边沿：跨批窗（4371µs）一次算准 */
    r = SpeedEst_Update(&se, &s[1], 1u, s[1].us);
    CHECK_NEAR(r, RAW(2, 4371.0f), 0.01f);

    /* 空拍：保持，且没到判停时限不许掉 0 */
    r = SpeedEst_Update(&se, NULL, 0u, s[1].us + 900u);
    CHECK_NEAR(r, RAW(2, 4371.0f), 0.01f);
}

/* 用例 3：静止判停与恢复直通。超时归 0；恢复转动后第一个读数直通真值，
 * 不从 0 慢慢爬（判停清了低通首值标记） */
static void TestStale_Zero(void)
{
    SpeedEst_t se;
    SpeedEst_Config_t c = { PPR, 0u, 30000u, 0.0f };    /* 判停用例与窗长无关，win_min 取 0 */
    SpeedEst_Init(&se, &c);

    SpeedEst_Sample_t s[2] = { { 2, 1603u }, { 4, 3206u } };
    float r = SpeedEst_Update(&se, s, 2u, s[1].us);
    CHECK_NEAR(r, RAW(2, 1603.0f), 1e-3f);

    /* 29999µs：还在 stale_us 之内，保持 */
    r = SpeedEst_Update(&se, NULL, 0u, s[1].us + 29999u);
    CHECK_NEAR(r, RAW(2, 1603.0f), 1e-3f);

    /* 30001µs：判停归 0 */
    r = SpeedEst_Update(&se, NULL, 0u, s[1].us + 30001u);
    CHECK(r == 0.0f);

    /* 恢复：新样本立刻出真值 */
    SpeedEst_Sample_t f[2] = { { 6, 35206u }, { 8, 36809u } };
    r = SpeedEst_Update(&se, f, 2u, f[1].us);
    CHECK_NEAR(r, RAW(2, 1603.0f), 1e-3f);
}

/* 用例 4：窗内方向反转带符号。窗短看瞬时速率、窗长看净值，都应是负数 */
static void TestReversal_Signed(void)
{
    /* cnt 序列 2,4,6,8,6,4,2,0：前半正爬后半反爬 */
    SpeedEst_Sample_t s[8];
    for (uint32_t i = 0u; i < 8u; i++)
    {
        s[i].us = (i + 1u) * 1000u;
        s[i].cnt = (i < 4u) ? (int32_t)(i + 1u) * 2
                            : (int32_t)(8u - i) * 2 - 2;
    }

    SpeedEst_t se;
    SpeedEst_Config_t c = { PPR, 0u, 30000u, 0.0f };    /* win_min=0：窗=最后两样本 */
    SpeedEst_Init(&se, &c);
    float r = SpeedEst_Update(&se, s, 8u, s[7].us);
    CHECK_NEAR(r, RAW(-2, 1000.0f), 1e-3f);

    SpeedEst_Config_t c7 = { PPR, 7000u, 30000u, 0.0f }; /* win_min=7ms：窗=整段净值 */
    SpeedEst_Init(&se, &c7);
    r = SpeedEst_Update(&se, s, 8u, s[7].us);
    CHECK_NEAR(r, RAW(-2, 7000.0f), 1e-3f);
}

/* 用例 5：µs 时间戳跨 0xFFFFFFFF 回绕。uint32 相减跨回绕天然正确，
 * 判停的"现在"也跨过去，不许误判停 */
static void TestTimestampWrap(void)
{
    SpeedEst_t se;
    SpeedEst_Config_t c = { PPR, 0u, 30000u, 0.0f };
    SpeedEst_Init(&se, &c);

    SpeedEst_Sample_t s[2] = { { 100u, 0xFFFFFF00u }, { 104u, 0x00000140u } };
    float r = SpeedEst_Update(&se, s, 2u, 0x00000240u);  /* now 也已跨回绕 */
    CHECK_NEAR(r, RAW(4, 576.0f), 1e-3f);

    r = SpeedEst_Update(&se, NULL, 0u, 0x00000340u);
    CHECK_NEAR(r, RAW(4, 576.0f), 1e-3f);
}

/* 用例 6：输出低通。rc=0 逐位透传；rc=1600µs 且相邻两次 Update 间隔恰
 * 1600µs 时 alpha 恰为 0.5，步进可手算：R1 → R1+Δ/2 → R1+3Δ/4 */
static void TestLpf_PassthroughAndRc(void)
{
    SpeedEst_t se;
    SpeedEst_Config_t c = { PPR, 0u, 30000u, 0.0f };
    SpeedEst_Init(&se, &c);

    SpeedEst_Sample_t a[2] = { { 2, 1600u }, { 4, 3200u } };
    float r = SpeedEst_Update(&se, a, 2u, a[1].us);
    CHECK_NEAR(r, RAW(2, 1600.0f), 1e-6f);
    SpeedEst_Sample_t b[2] = { { 6, 4800u }, { 10, 6400u } };
    r = SpeedEst_Update(&se, b, 2u, b[1].us);
    CHECK_NEAR(r, RAW(4, 1600.0f), 1e-6f);

    const float R1 = RAW(2, 1600.0f);   /* 30rpm 段真值（2 计数/1600µs） */
    const float R2 = RAW(4, 1600.0f);   /* 60rpm 段真值（4 计数/1600µs） */
    SpeedEst_Config_t cRc = { PPR, 0u, 30000u, 1600.0f };
    SpeedEst_Init(&se, &cRc);
    /* 单样本跨批窗序列，Update 间隔恒 1600µs → alpha 恒 0.5：
     * 30rpm 稳段滤不出变化，1200 计数阶跃后一次走一半 */
    SpeedEst_Sample_t st[6] = {
        { 2, 1600u }, { 4, 3200u }, { 6, 4800u },
        { 8, 6400u }, { 12, 8000u }, { 14, 9600u },
    };
    r = SpeedEst_Update(&se, &st[0], 1u, st[0].us);      /* 窗不够：保持 0 */
    CHECK(r == 0.0f);
    r = SpeedEst_Update(&se, &st[1], 1u, st[1].us);      /* 首值直通 */
    CHECK_NEAR(r, R1, 1e-6f);
    r = SpeedEst_Update(&se, &st[2], 1u, st[2].us);      /* 恒速段：滤了等于没滤 */
    CHECK_NEAR(r, R1, 1e-6f);
    r = SpeedEst_Update(&se, &st[3], 1u, st[3].us);
    CHECK_NEAR(r, R1, 1e-6f);
    r = SpeedEst_Update(&se, &st[4], 1u, st[4].us);      /* 阶跃：走一半 */
    CHECK_NEAR(r, R1 + (R2 - R1) * 0.5f, 1e-6f);
    r = SpeedEst_Update(&se, &st[5], 1u, st[5].us);      /* 回落：再走一半 */
    CHECK_NEAR(r, R1 + (R2 - R1) * 0.25f, 1e-6f);
}

/* 用例 7：退化输入不出 NaN/INF——从没喂过、单样本窗不够、同时间戳
 * 重复样本、全样本太老，四个口子都过一遍 */
static void TestDegenerate_Safe(void)
{
    SpeedEst_t se;
    SpeedEst_Config_t c = { PPR, 4000u, 30000u, 0.0f };
    SpeedEst_Init(&se, &c);

    CHECK(SpeedEst_Update(&se, NULL, 0u, 1000u) == 0.0f);

    SpeedEst_Sample_t s1 = { 2, 1000u };
    CHECK(SpeedEst_Update(&se, &s1, 1u, 1000u) == 0.0f);

    SpeedEst_Sample_t dup[2] = { { 2, 2000u }, { 2, 2000u } };
    float r = SpeedEst_Update(&se, dup, 2u, 2000u);
    CHECK(!isnan(r) && !isinf(r));

    r = SpeedEst_Update(&se, NULL, 0u, 2000u + 30001u);
    CHECK(r == 0.0f);
}

/* 用例 8：win_min=0 退化成"最近两样本"，一样精确 */
static void TestZeroWinMin_TwoSamples(void)
{
    SpeedEst_t se;
    SpeedEst_Config_t c = { PPR, 0u, 30000u, 0.0f };
    SpeedEst_Init(&se, &c);

    SpeedEst_Sample_t s[2] = { { 2, 1000u }, { 4, 2600u } };
    float r = SpeedEst_Update(&se, s, 2u, s[1].us);
    CHECK_NEAR(r, RAW(2, 1600.0f), 1e-3f);
}

/* 用例 9：滴喂回归（2026-10-21 S-C 板验翻车案）。1kHz 每拍只来一两滴时，
 * 本体跨调用只记上一批最新样本、窗龄凑不齐 → 抱死旧值，速度环追着冻结值
 * 乱撞。走滴喂包装后：换速必须跟得上（冻结实现停在旧值）、且输出持续
 * 更新（冻结实现 50 拍几乎不变化） */
static void TestDrip_NoFreezeAtSpeed(void)
{
    SpeedEst_t se;
    SpeedEstDrip_t dp;
    SpeedEst_Config_t c = { PPR, 4000u, 30000u, 0.0f };
    SpeedEst_Init(&se, &c);
    SpeedEstDrip_Init(&dp);

    SpeedEst_Sample_t batch[8];
    uint32_t interval = 874u;       /* 55rpm：沿距 874µs，每沿 2 计数 */
    uint32_t next_edge = interval;
    int32_t cnt = 0;
    float r = 0.0f;
    float last = -1000.0f;
    uint32_t switches = 0u;

    for (uint32_t k = 1u; k <= 120u; k++)
    {
        /* 拍点带 ±148µs 抖动：完美等间隔会让锚点年龄周期性恒定、读数恒成
         * 同一个浮点，"活跃度"判据就被合成数据自己锁死了——真实节拍有抖 */
        uint32_t now = k * 1000u + (k % 5u) * 37u;
        if (k == 61u)
        {
            interval = 601u;        /* 换 80rpm：沿距 601µs */
        }
        uint32_t n = 0u;
        while (next_edge <= now && n < 8u)
        {
            cnt += 2;
            batch[n].cnt = cnt;
            batch[n].us = next_edge;
            n++;
            /* 沿距带 ±30µs 抖动：便宜码盘的真实长相——完美等间隔会把锚点
             * 年龄锁死在两三个离散值上，读数恒成同一个浮点，判据失真 */
            next_edge += interval + (k % 11u) * 6u - 30u;
        }
        r = SpeedEstDrip_Update(&dp, &se, batch, n, now);
        if (k > 70u)                /* 换速后窗已翻新，统计读数活跃度 */
        {
            if (r != last)
            {
                switches++;
            }
            last = r;
        }
    }
    CHECK(switches >= 15u);         /* 冻结的旧实现 50 拍几乎不变化 */
    CHECK_NEAR(r, RAW(2, 601.0f), 8.0f);
}

/* 用例 10：滴喂停喂恢复。停 40ms 判停归 0；恢复后窗被裁到 2×win_min 内
 * 重新攒齐，~8ms 内回量程，不许拿跨停喂段的 ΔN/Δt 硬算 */
static void TestDrip_StallRecover(void)
{
    SpeedEst_t se;
    SpeedEstDrip_t dp;
    SpeedEst_Config_t c = { PPR, 4000u, 30000u, 0.0f };
    SpeedEst_Init(&se, &c);
    SpeedEstDrip_Init(&dp);

    SpeedEst_Sample_t batch[8];
    uint32_t interval = 874u;
    uint32_t next_edge = interval;
    int32_t cnt = 0;

    for (uint32_t k = 1u; k <= 60u; k++)    /* 先转起来 */
    {
        uint32_t now = k * 1000u;
        uint32_t n = 0u;
        while (next_edge <= now && n < 8u)
        {
            cnt += 2;
            batch[n].cnt = cnt;
            batch[n].us = next_edge;
            n++;
            next_edge += interval;
        }
        (void)SpeedEstDrip_Update(&dp, &se, batch, n, now);
    }

    /* 停喂 40ms（时钟照走、样本不来）：判停归 0 */
    float r = SpeedEstDrip_Update(&dp, &se, NULL, 0u, 60u * 1000u + 40000u);
    CHECK(r == 0.0f);

    /* 恢复滴喂：轮子停过，边沿从停喂结束后重新来；前几拍窗在重攒（读数
     * 从 0 爬回量程属正常），第 7~8 拍必须贴住真值——不许再拿跨停喂段的
     * 老锚点搭窗（跨批借样本只借新鲜度地平线以内的） */
    next_edge = 100u * 1000u + interval;
    float worst = 0.0f;
    for (uint32_t k = 1u; k <= 8u; k++)
    {
        uint32_t now = 100u * 1000u + k * 1000u;
        uint32_t n = 0u;
        while (next_edge <= now && n < 8u)
        {
            cnt += 2;
            batch[n].cnt = cnt;
            batch[n].us = next_edge;
            n++;
            next_edge += interval;
        }
        r = SpeedEstDrip_Update(&dp, &se, batch, n, now);
        if (k >= 7u && fabsf(r - RAW(2, 874.0f)) > worst)
        {
            worst = fabsf(r - RAW(2, 874.0f));
        }
    }
    CHECK(worst <= 8.0f);
}

/* 用例 11：拍内竞态的"未来时间戳"必须容忍。样本时间戳出自边沿中断、
 * now 出自任务拍首，任务取完时间戳才被中断盖章时最新样本比 now 快 1µs
 * ——无符号差分瞬间变 71.6 分钟"超龄"误判停归零（S-C 板验满屏扎零真凶）；
 * 有符号比较后正常出数且不留 stale 病痕 */
static void TestFutureStamp_Tolerated(void)
{
    SpeedEst_t se;
    SpeedEstDrip_t dp;
    SpeedEst_Config_t c = { PPR, 4000u, 30000u, 0.0f };
    SpeedEst_Init(&se, &c);
    SpeedEstDrip_Init(&dp);

    uint32_t now = 500000u;
    SpeedEst_Sample_t s[3] = {
        { 2, now - 5000u },     /* 窗锚点：对"未来样本"年龄恰 5001µs */
        { 4, now - 2500u },
        { 6, now + 1u },        /* 来自未来 1µs 的最新样本 */
    };
    float r = SpeedEstDrip_Update(&dp, &se, s, 3u, now);
    CHECK_NEAR(r, RAW(4, 5001.0f), 1e-3f);
    CHECK(se.dbg_stale_cnt == 0u);
}

void TestSpeedEst_Run(void)
{
    printf("[speed_est]\n");
    TestConst_NoQuantization();
    TestLowSpeed_SparseHold();
    TestStale_Zero();
    TestReversal_Signed();
    TestTimestampWrap();
    TestLpf_PassthroughAndRc();
    TestDegenerate_Safe();
    TestZeroWinMin_TwoSamples();
    TestDrip_NoFreezeAtSpeed();
    TestDrip_StallRecover();
    TestFutureStamp_Tolerated();
}
