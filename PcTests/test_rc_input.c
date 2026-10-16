/**
 * @file    test_rc_input.c
 * @brief   发送端输入适配单测（host 编译）——规格固化用例：
 *          轴适配数字表（中点零区/死区边界/满偏/整数取整）、Build 轴直通钳位、
 *          方向刀→关节映射、K_6→整车旋转、急停位、位图与 seq 透传、
 *          整帧清零纪律、NULL 安全、两拍消抖、双击检测。
 *          纯函数无替身依赖；期望值按 rc_input.c 同式整数手算。
 */
#include "rc_input.h"
#include "test_util.h"
#include <string.h>

/* 用 robot_config 默认标定构造的基准配置（轴速率=关节 100/旋转 500） */
static RcInput_Config_t DefaultCfg(void)
{
    RcInput_Config_t c;
    c.joy_center     = 2048u;
    c.joy_deadband   = 120u;
    c.key_joint_rate = 100;
    c.key_omega_rate = 500;
    return c;
}

static RcInput_Raw_t ZeroRaw(void)
{
    RcInput_Raw_t r;
    r.vx = 0;
    r.vy = 0;
    r.keys = 0u;
    r.estop = 0u;
    return r;
}

/* ---- RcInput_NormalizeAxis 数字表（PHYS 路径的轴适配） ---- */

/* 中点与死区内全零 */
static void TestAxisCenterZero(void)
{
    CHECK(RcInput_NormalizeAxis(2048u, 2048u, 120u) == 0);
    CHECK(RcInput_NormalizeAxis(2048u + 120u, 2048u, 120u) == 0);   /* 正向死区边缘 */
    CHECK(RcInput_NormalizeAxis(2048u - 120u, 2048u, 120u) == 0);   /* 负向死区边缘 */
    CHECK(RcInput_NormalizeAxis(2048u + 120u + 1u, 2048u, 120u) == 0);  /* 1000/1927=0 */
}

/* 线性段整数取整与满偏：正半程 904000/1927=469、负半程 -904000/1928=-468
 * （中点不在正中=两半程跨度不等，取整非对称）；端点=±满幅 */
static void TestAxisScale(void)
{
    CHECK(RcInput_NormalizeAxis(2048u + 720u, 2048u, 120u) == 311); /* 600000/1927 */
    CHECK(RcInput_NormalizeAxis(3072u, 2048u, 120u) == 469);
    CHECK(RcInput_NormalizeAxis(1024u, 2048u, 120u) == -468);
    CHECK(RcInput_NormalizeAxis(4095u, 2048u, 120u) == 1000);
    CHECK(RcInput_NormalizeAxis(0u, 2048u, 120u) == -1000);
}

/* ---- RcInput_Build（消费逻辑量快照） ---- */

/* 零输入：帧全零（含 omega 恒由 K_6 决定=零） */
static void TestCenterZero(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = ZeroRaw();
    RCPayload_t f;

    RcInput_Build(&cfg, &raw, 1u, &f);
    CHECK(f.vx == 0 && f.vy == 0 && f.omega == 0);
    CHECK(f.joint[0] == 0 && f.joint[1] == 0 && f.joint[2] == 0
          && f.joint[3] == 0 && f.joint[4] == 0);
    CHECK(f.keys == 0u && f.flags == 0u && f.seq == 1u);
}

/* 轴直通+钳位：±1000 内如实透传，超界钳回 ±1000（发送端纪律） */
static void TestBuildAxes(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = ZeroRaw();
    RCPayload_t f;

    raw.vx = 311;  raw.vy = -468;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == 311 && f.vy == -468);

    raw.vx = 1500;  raw.vy = -1500;     /* 越界输入（正常链路不该出现） */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == 1000 && f.vy == -1000);

    raw.vx = -32768;  raw.vy = 32767;   /* int16 极值也不得逸出 */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == -1000 && f.vy == 1000);
}

/* 方向刀→关节映射：A 刀=+rate、B 刀=−rate；同开关 A/B 同 1=矛盾归零；
 * 位图整字透传 */
static void TestDirectionKeys(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = ZeroRaw();
    RCPayload_t f;

    raw.keys = RCINPUT_KEY_K1_A | RCINPUT_KEY_K2_B | RCINPUT_KEY_K5_A;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.joint[0] == 100);       /* K_1 A = 大臂正 */
    CHECK(f.joint[1] == -100);      /* K_2 B = 小臂反 */
    CHECK(f.joint[2] == 0 && f.joint[3] == 0);
    CHECK(f.joint[4] == 100);       /* K_5 A = 手腕正 */
    CHECK(f.keys == raw.keys);      /* 位图整字透传 */

    raw.keys = RCINPUT_KEY_K1_A | RCINPUT_KEY_K1_B;     /* 同开关矛盾输入 */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.joint[0] == 0);         /* 失安全归零 */
    CHECK(f.keys == raw.keys);      /* 透传不吞 */

    cfg.key_joint_rate = 30;        /* 幅度随配置注入 */
    raw.keys = RCINPUT_KEY_K3_B;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.joint[2] == -30);
}

/* K_6 → 整车旋转（2026-10-16 拍板）：A=+rate、B=−rate、矛盾=0、幅度可注入 */
static void TestOmegaKeys(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = ZeroRaw();
    RCPayload_t f;

    raw.keys = RCINPUT_KEY_K6_A;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.omega == 500);
    CHECK(f.joint[0] == 0 && f.joint[4] == 0);      /* K_6 不进关节 */

    raw.keys = RCINPUT_KEY_K6_B;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.omega == -500);

    raw.keys = RCINPUT_KEY_K6_A | RCINPUT_KEY_K6_B; /* 矛盾失安全 */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.omega == 0);

    raw.keys = RCINPUT_KEY_K6_A | RCINPUT_KEY_K1_B; /* 与关节刀并行不互斥 */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.omega == 500 && f.joint[0] == -100);

    cfg.key_omega_rate = 750;                       /* 幅度随配置注入 */
    raw.keys = RCINPUT_KEY_K6_B;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.omega == -750);
}

/* 急停：estop=1 → flags bit0=1（车端闩锁语义）；estop=0 → flags=0 */
static void TestEstopFlag(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = ZeroRaw();
    RCPayload_t f;

    raw.estop = 1u;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK((f.flags & RC_FLAG_ESTOP) != 0u);

    raw.estop = 0u;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.flags == 0u);
}

/* 整帧清零纪律：入参 out 带垃圾值，建帧后不得残留 */
static void TestWholeFrameCleared(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = ZeroRaw();
    RCPayload_t f;

    memset(&f, 0xA5, sizeof f);
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.omega == 0 && f.vx == 0 && f.vy == 0);
    CHECK(f.flags == 0u && f.keys == 0u);
    for (int k = 0; k < ROBOT_CMD_JOINT_COUNT; k++)
    {
        CHECK(f.joint[k] == 0);
    }
}

/* NULL 安全：cfg/raw 为 NULL 时不写 out（预填哨兵不变）；out 为 NULL 不崩 */
static void TestNullSafety(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = ZeroRaw();
    RCPayload_t f;

    memset(&f, 0x5A, sizeof f);
    RcInput_Build(NULL, &raw, 0u, &f);
    CHECK(f.vx == (int16_t)0x5A5A);

    RcInput_Build(&cfg, NULL, 0u, &f);
    CHECK(f.vx == (int16_t)0x5A5A);

    RcInput_Build(&cfg, &raw, 0u, NULL);    /* 不崩即可 */
    /* 恢复正常调用一次，确认 out 未被 NULL 用例污染 */
    RcInput_Build(&cfg, &raw, 7u, &f);
    CHECK(f.seq == 7u);
}

/* 两拍一致消抖（16 位方向刀字）：稳定态直通、单拍毛刺滤除、双拍翻转、
 * 释放对称、位独立——含急停位（bit12）与高位方向刀（bit10/11） */
static void TestDebounce(void)
{
    /* 稳定态原样通过 */
    CHECK(RcInput_Debounce(0x0000u, 0x0000u, 0x0000u) == 0x0000u);
    CHECK(RcInput_Debounce(0xFFFFu, 0xFFFFu, 0xFFFFu) == 0xFFFFu);

    /* 单拍毛刺（上拍松、本拍瞬按）不上屏：低位方向刀 + 急停位各验一次 */
    CHECK(RcInput_Debounce(0x0000u, 0x0000u, RCINPUT_KEY_K1_A) == 0x0000u);
    CHECK(RcInput_Debounce(0x0000u, 0x0000u, RCINPUT_KEY_ESTOP_BIT) == 0x0000u);
    /* 连续两拍一致才翻转 */
    CHECK(RcInput_Debounce(0x0000u, RCINPUT_KEY_K6_B, RCINPUT_KEY_K6_B)
          == RCINPUT_KEY_K6_B);
    CHECK(RcInput_Debounce(0x0000u, RCINPUT_KEY_ESTOP_BIT, RCINPUT_KEY_ESTOP_BIT)
          == RCINPUT_KEY_ESTOP_BIT);
    /* 释放对称：单拍瞬松不翻转，两拍一致才清零 */
    CHECK(RcInput_Debounce(RCINPUT_KEY_K1_A, RCINPUT_KEY_K1_A, 0x0000u)
          == RCINPUT_KEY_K1_A);
    CHECK(RcInput_Debounce(RCINPUT_KEY_K1_A, 0x0000u, 0x0000u) == 0x0000u);
    /* 位独立：bit0 单拍毛刺不牵连 bit10（K_6 A）的双拍翻转 */
    CHECK(RcInput_Debounce(0x0000u, RCINPUT_KEY_K6_A,
                           RCINPUT_KEY_K6_A | RCINPUT_KEY_K1_A)
          == RCINPUT_KEY_K6_A);
    /* 未决位维持原态：bit1 翻转中不影响已稳定的 bit0=1 */
    CHECK(RcInput_Debounce(RCINPUT_KEY_K1_A, RCINPUT_KEY_K1_A,
                           RCINPUT_KEY_K1_A | RCINPUT_KEY_K1_B)
          == RCINPUT_KEY_K1_A);
}

/* 双击检测：单击不触发、窗内两按=事件保持 hold 拍、超窗重新武装、位独立 */
static void TestDclick(void)
{
    RcInput_DclickCfg_t cfg = { 600u, 3u };     /* 窗 600ms，事件保持 3 拍 */
    RcInput_DclickState_t st;

    /* 单击：按下+松开+超时，全程无事件 */
    RcInput_DclickReset(&st);
    CHECK(RcInput_DclickStep(&st, 1u, 0u, &cfg) == 0u);
    CHECK(RcInput_DclickStep(&st, 0u, 50u, &cfg) == 0u);
    CHECK(RcInput_DclickStep(&st, 0u, 700u, &cfg) == 0u);   /* >600ms 超时清武装 */
    /* 新的第一按后，窗内第二按 → 事件恰保持 3 拍 */
    CHECK(RcInput_DclickStep(&st, 1u, 800u, &cfg) == 0u);
    CHECK(RcInput_DclickStep(&st, 0u, 850u, &cfg) == 0u);
    CHECK(RcInput_DclickStep(&st, 1u, 900u, &cfg) == 1u);   /* 第二按当拍=事件 */
    CHECK(RcInput_DclickStep(&st, 1u, 910u, &cfg) == 1u);
    CHECK(RcInput_DclickStep(&st, 0u, 920u, &cfg) == 1u);
    CHECK(RcInput_DclickStep(&st, 0u, 930u, &cfg) == 0u);   /* 保持结束 */

    /* 超窗的第二按=新的第一按，不触发；再下一按与它配对成新双击 */
    RcInput_DclickReset(&st);
    CHECK(RcInput_DclickStep(&st, 1u, 0u, &cfg) == 0u);
    CHECK(RcInput_DclickStep(&st, 0u, 50u, &cfg) == 0u);
    CHECK(RcInput_DclickStep(&st, 1u, 700u, &cfg) == 0u);   /* 700-0>600 → 重新武装 */
    CHECK(RcInput_DclickStep(&st, 0u, 750u, &cfg) == 0u);
    CHECK(RcInput_DclickStep(&st, 1u, 800u, &cfg) == 1u);   /* 与 700ms 配对 */

    /* 保持窗内的再按下沿不吞事件：重新武装等下一轮 */
    CHECK(RcInput_DclickStep(&st, 0u, 810u, &cfg) == 1u);   /* 保持拍2 */
    CHECK(RcInput_DclickStep(&st, 0u, 820u, &cfg) == 1u);   /* 保持拍3 */
    CHECK(RcInput_DclickStep(&st, 1u, 830u, &cfg) == 0u);   /* 保持结束+新第一按 */
    CHECK(RcInput_DclickStep(&st, 0u, 860u, &cfg) == 0u);
    CHECK(RcInput_DclickStep(&st, 1u, 900u, &cfg) == 1u);   /* 830+900 配对 */

    /* NULL 安全 */
    CHECK(RcInput_DclickStep(NULL, 1u, 0u, &cfg) == 0u);
    RcInput_DclickReset(&st);
    RcInput_DclickReset(NULL);                              /* 不崩即可 */
    CHECK(RcInput_DclickStep(&st, 1u, 0u, NULL) == 0u);
}

void TestRcInput_Run(void)
{
    printf("-- rc_input\n");
    TestAxisCenterZero();
    TestAxisScale();
    TestCenterZero();
    TestBuildAxes();
    TestDirectionKeys();
    TestOmegaKeys();
    TestEstopFlag();
    TestWholeFrameCleared();
    TestNullSafety();
    TestDebounce();
    TestDclick();
}
