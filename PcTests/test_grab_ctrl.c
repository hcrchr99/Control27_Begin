/**
 * @file    test_grab_ctrl.c
 * @brief   抓取业务链 PC 单测：速率积分 / 软限位 / 急停保持 / 失联渐停 / 回绕
 *
 * 经真实 rc_cmd 层产快照（复用两级故障语义），时间全靠注入假 t_ms。
 * 输入即 joint[] 归一化速率（按键适配在发送端，车载链不消费 keys）。
 */
#include "test_util.h"
#include "grab_ctrl.h"
#include "rc_cmd.h"
#include <string.h>

/* ---- 小工具：喂一帧 + 推一拍（快照→Grab_Update），返回抓取态引用方便连写 ---- */
static Grab_State_t *g_sg;
static uint32_t g_t;

static void Seq_Frame_(const int8_t joint[5], uint8_t flags)
{
    RCPayload_t f;
    memset(&f, 0, sizeof f);
    f.seq = 1u;
    if (joint != NULL)
    {
        memcpy(f.joint, joint, sizeof f.joint);
    }
    f.flags = flags;
    RC_Cmd_Update(&f, g_t);
    RC_Cmd_t rc;
    RC_Cmd_GetCopy(&rc);
    Grab_Update(g_sg, rc.joint, g_t);   /* 任务口径：从快照摘出 joint[5] 再传 */
}

static void Seq_Null_(void)   /* 失联空帧驱动（看门狗路径） */
{
    RC_Cmd_Update(NULL, g_t);
    RC_Cmd_t rc;
    RC_Cmd_GetCopy(&rc);
    Grab_Update(g_sg, rc.joint, g_t);
}

void TestGrabCtrl_Run(void)
{
    printf("-- grab_ctrl --\n");
    RC_Cmd_Init();

    /* ===== Init：越限钳入软限位 ===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 0.0f, 200.0f, -5.0f, 90.0f, 90.0f };
        Grab_Init(&g, deg0);
        CHECK_NEAR(g.deg[0], 5.0, 1e-6);
        CHECK_NEAR(g.deg[1], 175.0, 1e-6);
        CHECK_NEAR(g.deg[2], 5.0, 1e-6);
        CHECK(!g.started);
    }

    /* ===== 首拍只对表不动 ===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 90, 90, 90, 90, 90 };
        Grab_Init(&g, deg0);
        g_sg = &g; g_t = 1000u;
        Seq_Frame_(NULL, 0);
        CHECK(g.started);
        CHECK_NEAR(g.deg[0], 90.0, 1e-6);              /* 首拍 dt=0 */
    }

    /* ===== 速率积分：满幅 100 = 180°/s ===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 90, 90, 90, 90, 90 };
        Grab_Init(&g, deg0);
        g_sg = &g;

        g_t = 1000u; Seq_Frame_(NULL, 0);              /* 对表 */
        g_t = 1100u;
        int8_t j[ROBOT_CMD_JOINT_COUNT] = { 100, 50, 0, 0, 0 };
        Seq_Frame_(j, 0);                              /* +100 → +18°；+50 → +9° */
        CHECK_NEAR(g.deg[0], 108.0, 1e-3);
        CHECK_NEAR(g.deg[1], 99.0, 1e-3);
        CHECK_NEAR(g.rate[0], 1.0, 1e-6);
        CHECK_NEAR(g.rate[1], 0.5, 1e-6);

        g_t = 1200u; Seq_Frame_(j, 0);                 /* 再 100ms 同速 */
        CHECK_NEAR(g.deg[0], 126.0, 1e-3);
    }

    /* ===== 零指令静默：无输入不动 ===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 90, 90, 90, 90, 90 };
        Grab_Init(&g, deg0);
        g_sg = &g;

        g_t = 2000u; Seq_Frame_(NULL, 0);
        g_t = 2100u; Seq_Frame_(NULL, 0);
        CHECK_NEAR(g.deg[3], 90.0, 1e-6);              /* 爪开合无指令不动 */
        CHECK_NEAR(g.rate[3], 0.0, 1e-6);
    }

    /* ===== 急停：角度保持不回零 ===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 90, 90, 90, 90, 90 };
        Grab_Init(&g, deg0);
        g_sg = &g;

        g_t = 3000u; Seq_Frame_(NULL, 0);
        g_t = 3100u;
        int8_t j[ROBOT_CMD_JOINT_COUNT] = { 100, 0, 0, 0, 0 };
        Seq_Frame_(j, 0);
        CHECK_NEAR(g.deg[0], 108.0, 1e-3);

        g_t = 3200u; Seq_Frame_(j, RC_FLAG_ESTOP);     /* 急停帧：指令被 rc_cmd 归零 */
        CHECK_NEAR(g.deg[0], 108.0, 1e-3);             /* 位置锁死，不回零 */
        CHECK_NEAR(g.rate[0], 0.0, 1e-6);

        g_t = 3300u; Seq_Frame_(NULL, 0);              /* 松急停（estop=0 帧解除） */
        g_t = 3400u; Seq_Frame_(NULL, 0);
        CHECK_NEAR(g.deg[0], 108.0, 1e-3);             /* 静默期不动 */
    }

    /* ===== 失联渐停：速率随快照线性滑落，位置无跳变 ===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 90, 90, 90, 90, 90 };
        Grab_Init(&g, deg0);
        g_sg = &g;

        g_t = 4000u; Seq_Frame_(NULL, 0);              /* 对表 */
        g_t = 4100u;
        int8_t j[ROBOT_CMD_JOINT_COUNT] = { 100, 0, 0, 0, 0 };
        Seq_Frame_(j, 0);                              /* +18° → 108，rate=1 */
        CHECK_NEAR(g.deg[0], 108.0, 1e-3);

        g_t = 4200u; Seq_Null_();                      /* 失联起点=本拍，rate 仍 1 */
        CHECK_NEAR(g.rate[0], 1.0, 1e-6);
        CHECK_NEAR(g.deg[0], 126.0, 1e-3);             /* +18° */

        g_t = 4300u; Seq_Null_();                      /* elapsed=100ms，rate=2/3 → +12° */
        CHECK_NEAR(g.rate[0], 2.0 / 3.0, 1e-5);
        CHECK_NEAR(g.deg[0], 138.0, 1e-3);

        g_t = 4400u; Seq_Null_();                      /* rate=1/3 → +6° */
        CHECK_NEAR(g.rate[0], 1.0 / 3.0, 1e-5);
        CHECK_NEAR(g.deg[0], 144.0, 1e-3);

        g_t = 4500u; Seq_Null_();                      /* 渐停完成，恒零 */
        CHECK_NEAR(g.rate[0], 0.0, 1e-6);
        CHECK_NEAR(g.deg[0], 144.0, 1e-3);             /* 总位移 = 离散梯形和，非跳变 */
    }

    /* ===== 软限位钳停：顶到头不越界 ===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 90, 90, 90, 90, 90 };
        Grab_Init(&g, deg0);
        g_sg = &g;

        g_t = 5000u; Seq_Frame_(NULL, 0);
        int8_t j[ROBOT_CMD_JOINT_COUNT] = { 100, 0, 0, 0, 0 };
        for (int k = 0; k < 200; k++)                  /* 满速 2s = 360° >> 行程 */
        {
            g_t += 10u;
            Seq_Frame_(j, 0);
        }
        CHECK_NEAR(g.deg[0], 175.0, 1e-6);             /* 精确停在软限位 */
        for (int k = 0; k < 20; k++)
        {
            g_t += 10u;
            Seq_Frame_(j, 0);
        }
        CHECK_NEAR(g.deg[0], 175.0, 1e-6);             /* 继续顶不累积 */
    }

    /* ===== 时基回绕：u32 无符号减法 dt 依然正确 ===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 90, 90, 90, 90, 90 };
        Grab_Init(&g, deg0);
        g_sg = &g;

        g_t = 0xFFFFFFF0u; Seq_Frame_(NULL, 0);        /* 对表在回绕点前 16ms */
        g_t = 0x00000028u;                             /* 回绕后 +40ms，dt=16+40=56ms */
        int8_t j[ROBOT_CMD_JOINT_COUNT] = { 100, 0, 0, 0, 0 };
        Seq_Frame_(j, 0);
        CHECK_NEAR(g.deg[0], 100.08, 1e-3);            /* 56ms × 180°/s = 10.08° */
    }
}
