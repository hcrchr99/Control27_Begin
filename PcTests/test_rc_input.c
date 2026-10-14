/**
 * @file    test_rc_input.c
 * @brief   发送端输入适配单测（host 编译）——规格固化用例：
 *          中点零区/死区边界、满偏 ±满幅、线性段整数取整、开关→关节映射、
 *          急停位、位图与 seq 透传、整帧清零纪律、NULL 安全。
 *          纯函数无替身依赖；期望值按 rc_input.c 同式整数手算。
 */
#include "rc_input.h"
#include "test_util.h"
#include <string.h>

/* 用 robot_config 默认标定（中点 2048 / 死区 120）构造的基准配置 */
static RcInput_Config_t DefaultCfg(void)
{
    RcInput_Config_t c;
    c.joy_center     = 2048u;
    c.joy_deadband   = 120u;
    c.key_joint_rate = 100;
    return c;
}

static RcInput_Raw_t CenterRaw(void)
{
    RcInput_Raw_t r;
    r.joy_x = 2048u;
    r.joy_y = 2048u;
    r.keys = 0u;
    r.estop = 0u;
    return r;
}

/* 中点与死区内全零，且 omega 恒 0、关节全零 */
static void TestCenterZero(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = CenterRaw();
    RCPayload_t f;

    RcInput_Build(&cfg, &raw, 1u, &f);
    CHECK(f.vx == 0 && f.vy == 0 && f.omega == 0);
    CHECK(f.joint[0] == 0 && f.joint[1] == 0 && f.joint[2] == 0
          && f.joint[3] == 0 && f.joint[4] == 0);
    CHECK(f.keys == 0u && f.flags == 0u && f.seq == 1u);
}

/* 死区边界：恰在死区边缘 = 0；刚出死区 = 最小可分辨步进 */
static void TestDeadbandEdge(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = CenterRaw();
    RCPayload_t f;

    raw.joy_x = 2048u + 120u;       /* 正向死区边缘 */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == 0);

    raw.joy_x = 2048u - 120u;       /* 负向死区边缘 */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == 0);

    raw.joy_x = 2048u + 120u + 1u;  /* 刚出死区 1 个 count：1000/1927 = 0（整数） */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == 0);

    raw.joy_x = 2048u + 720u;       /* 出死区 600 counts：600000/1927 = 311 */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == 311);
}

/* 物理端点 = ±满幅，两轴独立 */
static void TestFullScale(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = CenterRaw();
    RCPayload_t f;

    raw.joy_x = 4095u;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == 1000);
    CHECK(f.vy == 0);

    raw.joy_x = 0u;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == -1000);

    raw = CenterRaw();
    raw.joy_y = 0u;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vy == -1000);
    CHECK(f.vx == 0);
}

/* 线性段整数取整：正半程 904000/1927=469，负半程 -904000/1928=-468（非对称） */
static void TestMidpointQuantization(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = CenterRaw();
    RCPayload_t f;

    raw.joy_x = 3072u;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == 469);

    raw.joy_x = 1024u;
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.vx == -468);
}

/* 开关→关节映射：bit k → joint[k] = +rate；速率值随配置注入 */
static void TestKeysToJoints(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = CenterRaw();
    RCPayload_t f;

    raw.keys = 0x15u;   /* bit0/2/4 = K_1/K_3/K_5 按下 */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.joint[0] == 100 && f.joint[1] == 0 && f.joint[2] == 100
          && f.joint[3] == 0 && f.joint[4] == 100);
    CHECK(f.keys == 0x15u);     /* 位图透传 */

    cfg.key_joint_rate = -100;  /* 速率可配（反向方案定案后直接改宏） */
    RcInput_Build(&cfg, &raw, 0u, &f);
    CHECK(f.joint[0] == -100 && f.joint[2] == -100 && f.joint[4] == -100);
}

/* 急停：estop=1 → flags bit0=1（车端闩锁语义）；estop=0 → flags=0 */
static void TestEstopFlag(void)
{
    RcInput_Config_t cfg = DefaultCfg();
    RcInput_Raw_t raw = CenterRaw();
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
    RcInput_Raw_t raw = CenterRaw();
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
    RcInput_Raw_t raw = CenterRaw();
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

void TestRcInput_Run(void)
{
    printf("-- rc_input\n");
    TestCenterZero();
    TestDeadbandEdge();
    TestFullScale();
    TestMidpointQuantization();
    TestKeysToJoints();
    TestEstopFlag();
    TestWholeFrameCleared();
    TestNullSafety();
}
