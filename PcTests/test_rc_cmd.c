/**
 * @file    test_rc_cmd.c
 * @brief   RC_Cmd 单测（W2.2，host 编译）——规格固化用例：
 *          归一化/钳位、estop 立即零+闩锁+解除、失联 300ms 线性渐停、
 *          渐停被新帧打断、NULL 先于首帧、透传字段、Init 清态。
 *          时间全部由用例直接喂（now_ms 参数），无替身依赖。
 */
#include "rc_cmd.h"
#include "test_util.h"
#include <string.h>

/* 构帧助手：整帧清零再填（发送端纪律的用例侧复刻） */
static RCPayload_t MakeFrame(int16_t vx, int16_t vy, int16_t omega,
                             int8_t j0, int8_t j1, int8_t j2, int8_t j3, int8_t j4,
                             uint16_t keys, uint8_t flags)
{
    RCPayload_t f;
    memset(&f, 0, sizeof f);
    f.seq = 0u;   /* 透传用例单独设 */
    f.vx = vx;
    f.vy = vy;
    f.omega = omega;
    f.joint[0] = j0;
    f.joint[1] = j1;
    f.joint[2] = j2;
    f.joint[3] = j3;
    f.joint[4] = j4;
    f.keys = keys;
    f.flags = flags;
    return f;
}

/* Init 清态：GetCopy 必须是全零安全态，不是碰巧为零 */
static void TestInit(void)
{
    RC_Cmd_t c;
    RC_Cmd_Init();
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.valid == false);
    CHECK(c.estop == false);
    CHECK(c.vx == 0.0f && c.vy == 0.0f && c.omega == 0.0f);
    CHECK(c.joint[0] == 0.0f && c.joint[4] == 0.0f);
    CHECK(c.keys == 0u && c.seq == 0u && c.last_frame_ms == 0u);
}

/* 归一化与接收端防御钳位 + 透传字段 */
static void TestMappingAndClamp(void)
{
    RC_Cmd_t c;
    RC_Cmd_Init();

    RCPayload_t f = MakeFrame(500, -1000, 2000, 100, -127, 0, -100, 50, 0x00A5u, 0u);
    f.seq = 77u;
    RC_Cmd_Update(&f, 1000u);
    CHECK(RC_Cmd_GetCopy(&c));

    CHECK(c.valid == true && c.estop == false);
    CHECK_NEAR(c.vx, 0.5, 1e-6);        /* ÷1000 */
    CHECK_NEAR(c.vy, -1.0, 1e-6);       /* 满幅负向 */
    CHECK_NEAR(c.omega, 1.0, 1e-6);     /* 2000 超幅 → 接收端钳到 1.0 */
    CHECK_NEAR(c.joint[0], 1.0, 1e-6);
    CHECK_NEAR(c.joint[1], -1.0, 1e-6); /* int8 -127 超幅 → 钳 -1.0 */
    CHECK_NEAR(c.joint[2], 0.0, 1e-6);
    CHECK_NEAR(c.joint[3], -1.0, 1e-6);
    CHECK_NEAR(c.joint[4], 0.5, 1e-6);  /* 50/100 */
    CHECK(c.keys == 0x00A5u);           /* 透传 */
    CHECK(c.seq == 77u);                /* 透传 */
    CHECK(c.last_frame_ms == 1000u);
}

/* estop：立即归零 + valid 保持 true（链路活着，只是指令被强制清） */
static void TestEstopImmediateZero(void)
{
    RC_Cmd_t c;
    RC_Cmd_Init();

    RCPayload_t f = MakeFrame(800, 800, 800, 100, 100, 100, 100, 100, 0u, RC_FLAG_ESTOP);
    RC_Cmd_Update(&f, 500u);
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.estop == true && c.valid == true);
    CHECK(c.vx == 0.0f && c.vy == 0.0f && c.omega == 0.0f);
    CHECK(c.joint[0] == 0.0f && c.joint[4] == 0.0f);

    /* 松键：estop=0 帧 → 解除闩锁、指令恢复映射 */
    f.flags = 0u;
    RC_Cmd_Update(&f, 510u);
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.estop == false && c.valid == true);
    CHECK_NEAR(c.vx, 0.8, 1e-6);
}

/* estop 闩锁跨失联：看门狗路径不清 estop，恢复帧才清 */
static void TestEstopLatchThroughWatchdog(void)
{
    RC_Cmd_t c;
    RC_Cmd_Init();

    RCPayload_t f = MakeFrame(500, 0, 0, 0, 0, 0, 0, 0, 0u, RC_FLAG_ESTOP);
    RC_Cmd_Update(&f, 100u);
    RC_Cmd_Update(NULL, 600u);  /* 失联 */
    RC_Cmd_Update(NULL, 1500u); /* 渐停已到底 */
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.valid == false);
    CHECK(c.estop == true);     /* 闩锁不因链路死而解除 */
    CHECK(c.vx == 0.0f);

    RC_Cmd_Update(&f, 1600u);   /* 恢复：仍是 estop 帧 */
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.estop == true && c.valid == true);
}

/* 失联渐停：valid 立即 false；指令 300ms 线性滑落到零后恒零 */
static void TestDecayLinear(void)
{
    RC_Cmd_t c;
    RC_Cmd_Init();

    RCPayload_t f = MakeFrame(500, -400, 0, 50, 0, 0, 0, 0, 0u, 0u);
    RC_Cmd_Update(&f, 1000u);

    RC_Cmd_Update(NULL, 1400u); /* 失联检测 → 渐停起点，指令尚未缩 */
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.valid == false);
    CHECK_NEAR(c.vx, 0.5, 1e-6);

    RC_Cmd_Update(NULL, 1550u); /* +150ms：滑到一半 */
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK_NEAR(c.vx, 0.25, 1e-6);
    CHECK_NEAR(c.vy, -0.2, 1e-6);
    CHECK_NEAR(c.joint[0], 0.25, 1e-6);

    RC_Cmd_Update(NULL, 1699u); /* +299ms：残一丝 */
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.vx > 0.0f);

    RC_Cmd_Update(NULL, 1700u); /* +300ms：恰好到底 */
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.vx == 0.0f && c.vy == 0.0f);

    RC_Cmd_Update(NULL, 5000u); /* 之后恒零 */
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.vx == 0.0f && c.valid == false);
}

/* 渐停被新帧打断：恢复满幅并取消渐停，下次失联从新值起滑 */
static void TestDecayCancel(void)
{
    RC_Cmd_t c;
    RC_Cmd_Init();

    RCPayload_t f = MakeFrame(500, 0, 0, 0, 0, 0, 0, 0, 0u, 0u);
    RC_Cmd_Update(&f, 1000u);
    RC_Cmd_Update(NULL, 1400u); /* 渐停中 */
    RC_Cmd_Update(NULL, 1500u);

    f.vx = 800;
    RC_Cmd_Update(&f, 1600u);   /* 链路恢复 */
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.valid == true);
    CHECK_NEAR(c.vx, 0.8, 1e-6);

    RC_Cmd_Update(NULL, 2000u); /* 再次失联：从 0.8 起滑 */
    RC_Cmd_Update(NULL, 2150u);
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK_NEAR(c.vx, 0.4, 1e-6);
}

/* NULL 先于任何帧：安全零态，不崩 */
static void TestNullBeforeAnyFrame(void)
{
    RC_Cmd_t c;
    RC_Cmd_Init();
    RC_Cmd_Update(NULL, 5000u);
    RC_Cmd_Update(NULL, 5400u);
    CHECK(RC_Cmd_GetCopy(&c));
    CHECK(c.valid == false && c.vx == 0.0f);
}

/* GetCopy 参数防御 */
static void TestGetCopyNull(void)
{
    CHECK(RC_Cmd_GetCopy(NULL) == false);
}

void TestRcCmd_Run(void)
{
    printf("[rc_cmd]\n");
    TestInit();
    TestMappingAndClamp();
    TestEstopImmediateZero();
    TestEstopLatchThroughWatchdog();
    TestDecayLinear();
    TestDecayCancel();
    TestNullBeforeAnyFrame();
    TestGetCopyNull();
}
