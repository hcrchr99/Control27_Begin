/**
 * @file    test_lpf_ease.c
 * @brief   Lpf/Ease 单测（W2.1，host 编译）——alpha 钳位、首拍直通、
 *          收敛单调性、直通/保持两极；斜坡步进上限、精确到达、
 *          零/负步长原地不动（直达 bug 回归）。
 */
#include "ease.h"
#include "lpf.h"
#include "test_util.h"
#include <math.h>

static void TestLpfAlphaClamp(void)
{
    Lpf_t f;
    Lpf_Init(&f, -0.5f);
    CHECK_NEAR(f.alpha, 0.0, 1e-9);
    Lpf_Init(&f, 2.0f);
    CHECK_NEAR(f.alpha, 1.0, 1e-9);
}

static void TestLpfPrimingAndConvergence(void)
{
    Lpf_t f;
    Lpf_Init(&f, 0.5f);

    /* 首拍直通：不从 0 爬坡（上电假瞬态防护） */
    CHECK_NEAR(Lpf_Apply(&f, 5.0f), 5.0, 1e-9);

    /* 渐进回 0 基线（alpha=0.5 低通无瞬时回落；残差 = 5·2^(-拍数)，
     * 25 拍后 ≈1.5e-7） */
    for (int i = 0; i < 25; i++)
        (void)Lpf_Apply(&f, 0.0f);
    CHECK_NEAR(Lpf_Apply(&f, 0.0f), 0.0, 1e-6);

    /* 阶跃 0→10：5 → 7.5 → 8.75，单调收敛（容差含基线残差） */
    CHECK_NEAR(Lpf_Apply(&f, 10.0f), 5.0, 1e-6);
    CHECK_NEAR(Lpf_Apply(&f, 10.0f), 7.5, 1e-6);
    CHECK_NEAR(Lpf_Apply(&f, 10.0f), 8.75, 1e-6);

    float prev = 0.0f, out = 0.0f;
    for (int i = 0; i < 30; i++)
    {
        out = Lpf_Apply(&f, 10.0f);
        CHECK(out >= prev - 1e-9); /* 单调不减 */
        prev = out;
    }
    CHECK(fabs(out - 10.0f) < 0.001f); /* 足够逼近 */
}

static void TestLpfEnds(void)
{
    Lpf_t f;
    Lpf_Init(&f, 0.0f);
    (void)Lpf_Apply(&f, 1.0f);   /* 首拍直通=1 */
    CHECK_NEAR(Lpf_Apply(&f, 5.0f), 1.0, 1e-9); /* alpha=0：保持 */

    Lpf_Init(&f, 1.0f);
    (void)Lpf_Apply(&f, 1.0f);
    CHECK_NEAR(Lpf_Apply(&f, 5.0f), 5.0, 1e-9); /* alpha=1：直通 */
}

static void TestEaseStep(void)
{
    /* 上行：每步 max_step，末端精确到达不过冲 */
    float v = 0.0f;
    v = Ease_Step(v, 10.0f, 3.0f);
    CHECK_NEAR(v, 3.0, 1e-9);
    v = Ease_Step(v, 10.0f, 3.0f);
    CHECK_NEAR(v, 6.0, 1e-9);
    v = Ease_Step(v, 10.0f, 3.0f);
    CHECK_NEAR(v, 9.0, 1e-9);
    v = Ease_Step(v, 10.0f, 3.0f);
    CHECK_NEAR(v, 10.0, 1e-9); /* 剩 1 < 3 → 精确到达，不是 12 */

    /* 下行对称 */
    v = Ease_Step(0.0f, -10.0f, 3.0f);
    CHECK_NEAR(v, -3.0, 1e-9);
    v = Ease_Step(9.5f, -10.0f, 100.0f);
    CHECK_NEAR(v, -10.0, 1e-9); /* 一步可达 */

    /* 零/负/NaN 步长：原地不动（禁止一步直达——回归 ease.c 修正项） */
    CHECK_NEAR(Ease_Step(0.0f, 10.0f, 0.0f), 0.0, 1e-9);
    CHECK_NEAR(Ease_Step(0.0f, 10.0f, -1.0f), 0.0, 1e-9);
    CHECK_NEAR(Ease_Step(0.0f, 10.0f, NAN), 0.0, 1e-9);

    /* 目标不变：原样返回 */
    CHECK_NEAR(Ease_Step(7.0f, 7.0f, 3.0f), 7.0, 1e-9);
}

void TestLpfEase_Run(void)
{
    printf("[lpf/ease]\n");
    TestLpfAlphaClamp();
    TestLpfPrimingAndConvergence();
    TestLpfEnds();
    TestEaseStep();
}
