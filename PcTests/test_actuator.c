/**
 * @file    test_actuator.c
 * @brief   actuator 纯逻辑 PC 单测（W2.4）：状态机 / 斜坡 / 堵转确认窗 /
 *          滑动热累计 / u32 时戳回绕——时间与电流全靠注入，手推时戳验真
 */
#include <stdio.h>
#include <math.h>
#include "actuator.h"
#include "servo_stub.h"
#include "test_util.h"

/* 可注入电流源：g_cur 即"Power_GetJointCurrent 的返回值" */
static float g_cur;

static float CurSrc(void)
{
    return g_cur;
}

/* 默认实例：ID3、限位 10..170、ease 120°/s、堵转 1.5A/300ms、
 * 热保护 30s 窗 / 20s 上限 / 5s 冷却 */
static ActConfig_t CfgDefault(void)
{
    ActConfig_t c;
    c.servo_id = SERVO_ID3;
    c.min_deg = 10.0f;
    c.max_deg = 170.0f;
    c.ease_dps = 120.0f;
    c.get_current_a = CurSrc;
    c.stall_current_a = 1.5f;
    c.stall_confirm_ms = 300u;
    c.energize_window_ms = 30000u;
    c.energize_max_ms = 20000u;
    c.cooldown_ms = 5000u;
    return c;
}

/* 建好并对过表的实例（t 返回校准拍时戳） */
static Actuator_t Mk(uint32_t *t)
{
    Actuator_t a;
    ActConfig_t c = CfgDefault();
    (void)Act_Init(&a, &c);
    *t = 1000u;
    Act_Update(&a, *t);         /* 首拍校准 */
    return a;
}

/* 前进 n 拍（每拍 10ms） */
static void Run(Actuator_t *a, uint32_t *t, int n)
{
    for (int i = 0; i < n; i++)
    {
        *t += 10u;
        Act_Update(a, *t);
    }
}

void TestActuator_Run(void)
{
    printf("-- actuator --\n");

    /* ---- Init 参数校验 ---- */
    Actuator_t a;
    ActConfig_t c = CfgDefault();
    c.ease_dps = 0.0f;
    CHECK(!Act_Init(&a, &c));
    c = CfgDefault();
    c.min_deg = 170.0f;
    c.max_deg = 10.0f;
    CHECK(!Act_Init(&a, &c));
    c = CfgDefault();
    c.servo_id = SERVO_COUNT;
    CHECK(!Act_Init(&a, &c));
    c = CfgDefault();
    CHECK(Act_Init(&a, &c));
    CHECK(a.state == ACT_IDLE);
    CHECK(!Act_IsSettled(&a));
    CHECK_NEAR(a.cur_deg, 90.0f, 1e-6);

    /* ---- SetTarget 钳位 + NaN 忽略 ---- */
    uint32_t t;
    a = Mk(&t);
    Act_SetTarget(&a, 999.0f);
    CHECK_NEAR(a.target_deg, 170.0f, 1e-6);
    Act_SetTarget(&a, -5.0f);
    CHECK_NEAR(a.target_deg, 10.0f, 1e-6);
    Act_SetTarget(&a, NAN);
    CHECK(a.state == ACT_MOVING);       /* NaN 忽略：不改变此前 MOVING 态 */

    /* ---- 斜坡：90→150（60° @120°/s = 500ms），步进平滑到位 ---- */
    a = Mk(&t);
    Stub_Servo_Reset();
    Act_SetTarget(&a, 150.0f);
    CHECK(a.state == ACT_MOVING);
    CHECK(!Act_IsSettled(&a));
    Run(&a, &t, 11);                    /* 110ms：应到 90+13.2=103.2 */
    CHECK_NEAR(Stub_Servo_LastDeg, 103.2f, 0.3f);
    /* 中途任一拍都不得越过目标 */
    CHECK(Stub_Servo_LastDeg < 150.0f);
    Run(&a, &t, 40);                    /* 累计 510ms > 500ms */
    CHECK(a.state == ACT_HOLDING);
    CHECK(Act_IsSettled(&a));
    CHECK_NEAR(Stub_Servo_LastDeg, 150.0f, 1e-6);   /* 到点精确落位 */
    /* HOLDING 恒发：每拍仍写 servo */
    int set0 = Stub_Servo_SetCalls;
    Run(&a, &t, 3);
    CHECK(Stub_Servo_SetCalls == set0 + 3);

    /* ---- MOVING 中重设目标：反向重算斜坡 ---- */
    a = Mk(&t);
    Stub_Servo_Reset();
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 25);                    /* 250ms：约 120° */
    CHECK_NEAR(Stub_Servo_LastDeg, 120.0f, 1.5f);
    Act_SetTarget(&a, 90.0f);           /* 反向，30° = 250ms */
    CHECK(a.state == ACT_MOVING);
    Run(&a, &t, 10);                    /* +100ms：应向 90 方向回走 */
    CHECK(Stub_Servo_LastDeg < 121.0f);
    Run(&a, &t, 20);                    /* 满 250ms */
    CHECK(a.state == ACT_HOLDING);
    CHECK_NEAR(Stub_Servo_LastDeg, 90.0f, 1e-6);

    /* ---- Release：卸力后不再发角度 ---- */
    a = Mk(&t);
    Stub_Servo_Reset();
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 60);                    /* 到位 */
    Act_Release(&a);
    CHECK(a.state == ACT_RELEASED);
    CHECK(!Act_IsSettled(&a));
    CHECK(Stub_Servo_Released[SERVO_ID3]);
    int set1 = Stub_Servo_SetCalls;
    Run(&a, &t, 5);
    CHECK(Stub_Servo_SetCalls == set1); /* RELEASED 不写 servo */

    /* ---- RELEASED 后 SetTarget 重新上力走斜坡 ---- */
    Act_SetTarget(&a, 90.0f);
    CHECK(a.state == ACT_MOVING);
    Run(&a, &t, 1);                         /* 首个 Update 拍恢复发脉冲 */
    CHECK(!Stub_Servo_Released[SERVO_ID3]); /* SetAngle 恢复发脉冲 */

    /* ---- 堵转：持续超阈值确认窗坐实；自动卸力 + 锁存 ----
     * 确认窗从首次超阈拍起算（t=1010），第 31 拍（t=1310）差值满 300ms */
    a = Mk(&t);
    Stub_Servo_Reset();
    g_cur = 2.0f;                       /* > 1.5A 阈值 */
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 30);                    /* 300 拍内差值最大 290ms：未满窗 */
    CHECK(!Act_IsStalled(&a));
    CHECK(a.state == ACT_MOVING);       /* 未卸力 */
    Run(&a, &t, 1);                     /* 差值 300ms：到窗 */
    CHECK(Act_IsStalled(&a));
    CHECK(a.state == ACT_RELEASED);     /* 立即卸力 */
    CHECK(Stub_Servo_Released[SERVO_ID3]);

    /* ---- 瞬时冲击不误触发 ---- */
    a = Mk(&t);
    g_cur = 2.0f;
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 5);                     /* 50ms 冲击 */
    g_cur = 0.5f;
    Run(&a, &t, 50);                    /* 回落后再走 500ms */
    CHECK(!Act_IsStalled(&a));
    CHECK(a.state == ACT_HOLDING);

    /* ---- NaN 电流不误触发（采样通道异常失安全） ---- */
    a = Mk(&t);
    g_cur = NAN;
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 60);
    CHECK(!Act_IsStalled(&a));
    CHECK(a.state == ACT_HOLDING);

    /* ---- stall_threshold ≤ 0 = 判据禁用 ---- */
    a = Mk(&t);
    a.cfg.stall_current_a = 0.0f;
    g_cur = 3.0f;
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 60);
    CHECK(!Act_IsStalled(&a));

    /* ---- stalled 锁存保到下个 SetTarget ----
     * 触发时 MOVING 已走 310ms（cur≈127.2），回 90 差 37.2° = 310ms 斜坡 */
    a = Mk(&t);
    g_cur = 2.0f;
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 31);                    /* 差值满 300ms：触发堵转 */
    CHECK(Act_IsStalled(&a));
    g_cur = 0.3f;
    Act_SetTarget(&a, 90.0f);           /* 新动作清锁存重新起判 */
    CHECK(!Act_IsStalled(&a));
    CHECK(a.state == ACT_MOVING);
    Run(&a, &t, 32);                    /* 310ms 斜坡走完 + 电流正常不触发 */
    CHECK(!Act_IsStalled(&a));
    CHECK(a.state == ACT_HOLDING);

    /* ---- 热保护：20s 供电坐实；只报告不强制卸力 ---- */
    a = Mk(&t);
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 1950);                  /* 19.5s：未到上限 */
    CHECK(!Act_NeedsCooldown(&a));
    Run(&a, &t, 60);                    /* +0.6s：累计 20.1s */
    CHECK(Act_NeedsCooldown(&a));
    CHECK(a.state == ACT_HOLDING);      /* 不强制卸力 */
    CHECK(!Stub_Servo_Released[SERVO_ID3]);

    /* ---- 冷却：卸力 5s 自动解除，累计清零重计 ----
     * idle_t0 在卸力后首拍记起点，故需 501 拍差值才满 5000ms */
    Act_Release(&a);
    Run(&a, &t, 490);                   /* 4.9s：未满冷却 */
    CHECK(Act_NeedsCooldown(&a));
    Run(&a, &t, 11);                    /* 累计 501 拍：满 5s */
    CHECK(!Act_NeedsCooldown(&a));
    CHECK(a.state == ACT_RELEASED);     /* 冷却不改变卸力态 */
    /* 解除后重新供电：整窗内不足上限不再触发 */
    Act_SetTarget(&a, 90.0f);
    Run(&a, &t, 1500);                  /* 15s 供电 */
    CHECK(!Act_NeedsCooldown(&a));

    /* ---- 供电打断冷却计时 ---- */
    a = Mk(&t);
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 2100);                  /* 21s：触发 */
    CHECK(Act_NeedsCooldown(&a));
    Act_Release(&a);
    Run(&a, &t, 300);                   /* 冷却 3s */
    Act_SetTarget(&a, 90.0f);           /* 重新上力（hot 仍在） */
    Run(&a, &t, 100);                   /* 供电 1s：冷却被打断 */
    Act_Release(&a);
    Run(&a, &t, 499);                   /* 卸力 4.99s：不足 5s（中途被打断过） */
    CHECK(Act_NeedsCooldown(&a));
    Run(&a, &t, 2);                     /* 满 501 拍：5s（本次连续） */
    CHECK(!Act_NeedsCooldown(&a));

    /* ---- Release 期间不累计：旧桶随时间滑出 ---- */
    a = Mk(&t);
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 1200);                  /* 12s 供电 */
    Act_Release(&a);
    Run(&a, &t, 3000);                  /* 卸力 30s：桶全滑出 */
    Act_SetTarget(&a, 90.0f);
    Run(&a, &t, 1000);                  /* 再供电 10s */
    CHECK(!Act_NeedsCooldown(&a));

    /* ---- 热保护禁用（energize_max_ms=0）：长供电不触发 ---- */
    a = Mk(&t);
    a.cfg.energize_max_ms = 0u;
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 6000);                  /* 60s */
    CHECK(!Act_NeedsCooldown(&a));

    /* ---- 无电流源实例（get_current_a=NULL）：判据禁用但热保护照常 ---- */
    a = Mk(&t);
    a.cfg.get_current_a = NULL;
    g_cur = 3.0f;                       /* 有源也不该被读到 */
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 60);
    CHECK(!Act_IsStalled(&a));
    Run(&a, &t, 2100);                  /* 累计供电 21s+ */
    CHECK(Act_NeedsCooldown(&a));

    /* ---- u32 时戳回绕：MOVING 跨 0 点斜坡照常 ---- */
    a = Mk(&t);
    g_cur = 0.0f;                       /* 回绕段无堵转干扰（上组残留清掉） */
    t = 0xFFFFFFFFu - 1000u;
    Act_Update(&a, t);                  /* 回绕前对表 */
    Act_SetTarget(&a, 150.0f);          /* 500ms 斜坡 */
    Run(&a, &t, 25);                    /* 250ms：跨过 0 点 */
    CHECK(a.state == ACT_MOVING);
    CHECK(Stub_Servo_LastDeg > 90.0f);  /* 已走出一段 */
    CHECK(Stub_Servo_LastDeg < 150.0f);
    Run(&a, &t, 26);                    /* 满 500ms（回绕后） */
    CHECK(a.state == ACT_HOLDING);
    CHECK_NEAR(Stub_Servo_LastDeg, 150.0f, 1e-6);

    /* ---- 长眠恢复：整窗过去后热累计全清；冷却计时随后照常走 ---- */
    a = Mk(&t);
    g_cur = 0.0f;
    Act_SetTarget(&a, 150.0f);
    Run(&a, &t, 2100);                  /* 21s 供电：触发 */
    CHECK(Act_NeedsCooldown(&a));
    Act_Release(&a);
    t += 40000u;                        /* 静置 40s（一次大步长） */
    Act_Update(&a, t);                  /* 此拍清桶 + 开始冷却计时 */
    CHECK(Act_NeedsCooldown(&a));       /* 冷却 5s 尚未走完 */
    Run(&a, &t, 510);                   /* 冷却满 5s */
    CHECK(!Act_NeedsCooldown(&a));
    Act_SetTarget(&a, 90.0f);
    Run(&a, &t, 100);                   /* 重新供电 1s */
    CHECK(!Act_NeedsCooldown(&a));
}
