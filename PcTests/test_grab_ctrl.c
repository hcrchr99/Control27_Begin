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

    /* ===== Init：越限钳入软限位（2026-10-12 限位占位加宽到 [0,180]）===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 0.0f, 200.0f, -5.0f, 90.0f, 90.0f };
        Grab_Init(&g, deg0);
        CHECK_NEAR(g.deg[0], 0.0, 1e-6);
        CHECK_NEAR(g.deg[1], 180.0, 1e-6);
        CHECK_NEAR(g.deg[2], 0.0, 1e-6);
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
        CHECK_NEAR(g.deg[0], 180.0, 1e-6);             /* 精确停在软限位 */
        for (int k = 0; k < 20; k++)
        {
            g_t += 10u;
            Seq_Frame_(j, 0);
        }
        CHECK_NEAR(g.deg[0], 180.0, 1e-6);             /* 继续顶不累积 */
    }

    /* ===== 姿态宏：初始位→展开位，速率钳制+精确收敛+爪保持 ===== */
    {
        Grab_State_t g;
        float deg0[ROBOT_CMD_JOINT_COUNT] = { 0, 180, 92, 63, 120 };  /* 初始位，爪两路非中位 */
        Grab_Init(&g, deg0);
        /* 展开：臂三关节去 (30,148,92)，爪两路冻结当前值（63/120） */
        const float arm3[3] = { 30.0f, 148.0f, 92.0f };
        float tgt[ROBOT_CMD_JOINT_COUNT] = { arm3[0], arm3[1], arm3[2],
                                             g.deg[3], g.deg[4] };
        Grab_PoseMove_t m;
        Grab_PoseMoveStart(&m, tgt, 60.0f, 10000u);

        uint32_t t = 10000u;
        float rate[ROBOT_CMD_JOINT_COUNT];
        bool stepping = true;
        int ticks = 0;
        float max_step = 0.0f;
        while (stepping && ticks < 200)
        {
            t += 50u;                                  /* 20Hz 拍 */
            stepping = Grab_PoseMoveStep(&m, &g, t, rate);
            if (stepping)
            {
                /* 每关节每拍位移 ≤ dps·dt（60°/s×50ms=3°） */
                for (int i = 0; i < ROBOT_CMD_JOINT_COUNT; i++)
                {
                    float move = rate[i] * 180.0f * 0.05f;
                    if (fabsf(move) > max_step)
                    {
                        max_step = fabsf(move);
                    }
                    g.deg[i] += move;                  /* 手工复刻 Grab_Update 积分 */
                }
            }
            ticks++;
        }
        CHECK(!stepping);                              /* 收摊 */
        CHECK(ticks < 60);                             /* 92°行程/60°/s≈1.5s≈30 拍量级 */
        CHECK(max_step <= 3.0f + 1e-4);                /* 速率钳制无超调 */
        CHECK_NEAR(g.deg[0], 30.0, 0.1);
        CHECK_NEAR(g.deg[1], 148.0, 0.1);
        CHECK_NEAR(g.deg[2], 92.0, 0.1);
        CHECK_NEAR(g.deg[3], 63.0, 1e-6);              /* 爪开合保持原值 */
        CHECK_NEAR(g.deg[4], 120.0, 1e-6);             /* 爪旋转保持原值 */

        /* 到位后再 Step = 无输出 */
        CHECK(!Grab_PoseMoveStep(&m, &g, t + 50u, rate));
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

    /* ===== 姿态宏：目标越限位钳入（Start 规则同 Grab_Init）===== */
    {
        Grab_PoseMove_t m;
        float tgt[ROBOT_CMD_JOINT_COUNT] = { 200.0f, -30.0f, 92.0f, 90.0f, 90.0f };
        Grab_PoseMoveStart(&m, tgt, 60.0f, 0u);
        CHECK_NEAR(m.tgt[0], 180.0, 1e-6);
        CHECK_NEAR(m.tgt[1], 0.0, 1e-6);
        CHECK(m.active);
        /* dps 非法值兜底回满幅 */
        Grab_PoseMoveStart(&m, tgt, 0.0f, 0u);
        CHECK_NEAR(m.dps, 180.0, 1e-6);
    }
}
