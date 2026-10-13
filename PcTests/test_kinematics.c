/**
 * @file    test_kinematics.c
 * @brief   kinematics PC 单测（§3.9 验收构想的 PC 部分）
 *
 * 预期值来源：手推闭式几何（web_sim links.json CAD 同源常量），非实现
 * 自洽——FK 零位爪尖、roll=90° 横移量、开合方向、预设点位可达性均为
 * 独立算出的硬数；FK↔IK 往返再加网格自洽覆盖。
 */
#include "test_util.h"
#include "kinematics.h"
#include "robot_config.h"
#include <math.h>

#define TIP_EPS  0.1f   /* 往返容差 mm（§3.9 验收：|Δ|<0.1mm） */
#define DEG_EPS  0.2f   /* 手算预期角容差（手推保留两位小数） */

static void run(void);

void TestKinematics_Run(void)
{
    printf("-- kinematics\n");
    run();
}

/* ---- 硬数（手推） ---- */
/* FK 全零角：爪尖 = S+U+R0+V+G = (319.800, 104.198)，横向 = 56.550 */
#define EXP_TIP0_X   319.800
#define EXP_TIP0_Z   104.198
#define EXP_TIP0_Y   56.550
/* roll=90°（腕 90°）：开合方向面内投影 = perp(a) = (−0.489405, 0.872057)，
 * 夹取点横向 = 46.593 + (a_z·w_x − a_x·w_z) = 124.057，w=V+G */
#define EXP_OPEN90_X (-0.489405)
#define EXP_OPEN90_Z 0.872057
#define EXP_TIPY_90  124.057
/* OPEN 逆解 (240,150) roll=90 腕90：el1=−37.33° el2=68.10° → 舵机 127.33/21.90 */
#define EXP_DESC_S1  127.33
#define EXP_DESC_S2  21.90
/* 下插构造：from(240,150) drop=60 roll=90 → to=(269.364, 97.677) */
#define EXP_DROP_X   269.364
#define EXP_DROP_Z   97.677

static bool tip_near(const Kin_Pose_t *p, double x, double z, double eps)
{
    return fabs(p->x_mm - x) <= eps && fabs(p->z_mm - z) <= eps;
}

/* 插补收敛检查用的最后位姿（TOOL 直线用例内填充） */
static Kin_Pose_t p_last;

static void run(void)
{
    /* ---- FK 零位（建模姿态） ---- */
    {
        const float deg0[3] = { 90.0f, 90.0f, 90.0f };
        Kin_Pose_t p;
        Kin_Fk(deg0, 0.0f, &p);
        CHECK(tip_near(&p, EXP_TIP0_X, EXP_TIP0_Z, TIP_EPS));
        CHECK_NEAR(p.y_mm, EXP_TIP0_Y, 0.01);
        /* roll=0：开合纯横向，面内投影为零向量 */
        CHECK_NEAR(p.open_x, 0.0, 1e-6);
        CHECK_NEAR(p.open_z, 0.0, 1e-6);
    }

    /* ---- 开合方向（只随 roll 变） ---- */
    {
        float u[2];
        Kin_OpeningDir(90.0f, u);
        CHECK_NEAR(u[0], EXP_OPEN90_X, 1e-4);
        CHECK_NEAR(u[1], EXP_OPEN90_Z, 1e-4);
        /* 面内投影长度 = sin(roll) */
        for (int a = 0; a <= 180; a += 30)
        {
            float v[2];
            Kin_OpeningDir((float)a, v);
            CHECK_NEAR(sqrt(v[0] * v[0] + v[1] * v[1]), sin(a * 3.14159265 / 180.0), 1e-4);
        }
    }

    /* ---- TOOL 模式 FK↔IK 网格往返 ---- */
    {
        for (float x = 150.0f; x <= 300.0f; x += 25.0f)
        {
            for (float z = 60.0f; z <= 240.0f; z += 30.0f)
            {
                for (float pitch = -60.0f; pitch <= 30.0f; pitch += 30.0f)
                {
                    float tip[2] = { x, z };
                    float deg[3];
                    if (!Kin_IkTool(tip, pitch, deg))
                    {
                        continue;   /* 网格含不可达点，逆解拒绝即过 */
                    }
                    Kin_Pose_t p;
                    Kin_Fk(deg, 0.0f, &p);
                    CHECK(tip_near(&p, x, z, TIP_EPS));
                }
            }
        }
    }

    /* ---- OPEN 模式 FK↔IK 网格往返（roll/腕固定组合） ---- */
    {
        float rolls[3] = { 30.0f, 90.0f, 150.0f };
        float wrists[3] = { 60.0f, 90.0f, 120.0f };
        for (int r = 0; r < 3; r++)
        {
            for (int w = 0; w < 3; w++)
            {
                for (float x = 180.0f; x <= 300.0f; x += 30.0f)
                {
                    for (float z = 60.0f; z <= 220.0f; z += 40.0f)
                    {
                        float tip[2] = { x, z };
                        float deg[3];
                        if (!Kin_IkOpen(tip, rolls[r], wrists[w], deg))
                        {
                            continue;
                        }
                        CHECK_NEAR(deg[2], wrists[w], 1e-4); /* 腕角恒定带回 */
                        Kin_Pose_t p;
                        Kin_Fk(deg, rolls[r], &p);
                        CHECK(tip_near(&p, x, z, TIP_EPS));
                    }
                }
            }
        }
    }

    /* ---- OPEN 逆解对手算硬数（roll=90 腕90 @ (240,150)） ---- */
    {
        float tip[2] = { 240.0f, 150.0f };
        float deg[3];
        CHECK(Kin_IkOpen(tip, 90.0f, 90.0f, deg));
        CHECK_NEAR(deg[0], EXP_DESC_S1, DEG_EPS);
        CHECK_NEAR(deg[1], EXP_DESC_S2, DEG_EPS);
        Kin_Pose_t p;
        Kin_Fk(deg, 90.0f, &p);
        CHECK(tip_near(&p, 240.0, 150.0, TIP_EPS));
        CHECK_NEAR(p.y_mm, EXP_TIPY_90, 0.05);  /* roll=90 横移 124mm：底盘对准补偿量 */
    }

    /* ---- 拒绝路径：不可达 / 伸直奇异 / 越软限位 ---- */
    {
        float far_tip[2] = { 400.0f, 400.0f };
        float deg[3];
        CHECK(!Kin_IkTool(far_tip, 0.0f, deg));         /* 臂长之外 */

        /* 伸直：腕点放在 |U|+|P|−微米 的方向上（r≈223.0 → cosΔ≈0.998>0.98） */
        float wx = 48.900 + 223.0 * 0.93969;            /* 20° 方向腕点 */
        float wz = 46.198 + 223.0 * 0.34202;
        float t[2] = { wx + 94.434, wz };               /* 沿 0° 爪向补安装向量 */
        CHECK(!Kin_IkTool(t, 0.0f, deg));

        float near_tip[2] = { 60.0f, 60.0f };           /* 肘角需要 138°>限位 */
        CHECK(!Kin_IkTool(near_tip, 0.0f, deg));
    }

    /* ---- 预设点位：三组全部可达且往返闭合 ---- */
    {
        const float presets[3][3] = ROBOT_ARM_PRESET_LIST;
        for (int i = 0; i < ROBOT_ARM_PRESET_COUNT; i++)
        {
            float tip[2] = { presets[i][0], presets[i][1] };
            float deg[3];
            bool ok = Kin_IkTool(tip, presets[i][2], deg);
            CHECK(ok);
            if (!ok)
            {
                continue;
            }
            Kin_Pose_t p;
            Kin_Fk(deg, 0.0f, &p);
            CHECK(tip_near(&p, presets[i][0], presets[i][1], TIP_EPS));
        }
    }

    /* ---- 直线插补 TOOL：全程推进收敛终点、中点在线上 ---- */
    {
        Kin_Line_t l = { 0 };
        Kin_LineReq_t req = {
            .mode = KIN_MODE_TOOL,
            .from_x_mm = 260.0f, .from_z_mm = 170.0f,
            .to_x_mm = 300.0f, .to_z_mm = 110.0f,
            .tool_pitch_deg = 0.0f,
        };
        CHECK(Kin_LineStart(&l, &req, 1000));
        CHECK(l.active);

        float deg[3];
        uint32_t now = 1000;
        bool any_mid = false;
        while (l.active)
        {
            CHECK(Kin_LineStep(&l, now, deg));  /* 拒绝=数值异常，测试挂 */
            Kin_Pose_t p;
            Kin_Fk(deg, 0.0f, &p);
            /* 逐拍爪尖到直线段距离 < 0.2mm（终点容差放宽到往返双倍） */
            float vx = 40.0f, vz = -60.0f;      /* to-from */
            float t = ((p.x_mm - 260.0f) * vx + (p.z_mm - 170.0f) * vz) / (vx * vx + vz * vz);
            if (t > 0.01f && t < 0.99f)
            {
                float cx = 260.0f + vx * t, cz = 170.0f + vz * t;
                CHECK_NEAR(p.x_mm - cx, 0.0, 0.2);
                CHECK_NEAR(p.z_mm - cz, 0.0, 0.2);
                any_mid = true;
            }
            now += 10;
        }
        CHECK(any_mid);
        Kin_Fk(deg, 0.0f, &p_last);
        CHECK(tip_near(&p_last, 300.0, 110.0, 2.0 * TIP_EPS));
        CHECK(!l.active);                       /* 终点拍后收摊 */
        CHECK(!Kin_LineStep(&l, now, deg));     /* 收摊后再 Step = 无输出 */
    }

    /* ---- 直线插补 OPEN：下插构造 + 预扫拒绝 ---- */
    {
        float from[2] = { 240.0f, 150.0f };
        Kin_LineReq_t req;
        Kin_LineDescend(&req, from, 60.0f, 90.0f, 90.0f);
        CHECK_NEAR(req.to_x_mm, EXP_DROP_X, 0.01);
        CHECK_NEAR(req.to_z_mm, EXP_DROP_Z, 0.01);

        Kin_Line_t l = { 0 };
        CHECK(Kin_LineStart(&l, &req, 0));
        float deg[3];
        uint32_t now = 0;
        while (l.active)
        {
            CHECK(Kin_LineStep(&l, now, deg));
            now += 10;
        }
        Kin_Pose_t p;
        Kin_Fk(deg, 90.0f, &p);
        CHECK(tip_near(&p, EXP_DROP_X, EXP_DROP_Z, 2.0 * TIP_EPS));

        /* 预扫拒绝：终点远超臂长，Start 直接 false 且不激活 */
        Kin_Line_t l2 = { 0 };
        Kin_LineReq_t bad = {
            .mode = KIN_MODE_TOOL,
            .from_x_mm = 200.0f, .from_z_mm = 150.0f,
            .to_x_mm = 450.0f, .to_z_mm = 50.0f,
            .tool_pitch_deg = 0.0f,
        };
        CHECK(!Kin_LineStart(&l2, &bad, 0));
        CHECK(!l2.active);
    }

    /* ---- NORMAL 模式：与用户 claw_traj.py 轨迹点对表（traj_-90.json）----
     * 竖直下压（dir=−90°，Σ_urdf=140.5）：爪心三点，θ 为 URDF 口径，
     * 固件舵机角 = θ + 90。用户数值迭代 vs 固件闭式 2R：容差 0.05°——
     * 用户工具 Σ 量化在 140.5°（细步长 0.02°），固件用精确
     * Σ=−90−α(n0)=140.538°，差值按三关节分摊 ~0.02° */
    {
        struct { float x, z, t1, t2, t4; } ref[3] = {
            { 129.998f, 73.199f, -15.9606f, 82.7723f, 73.6883f },
            { 130.019f, 38.199f,  -5.1302f, 85.6647f, 59.9655f },
            { 130.041f,  3.199f,   7.8483f, 82.7558f, 49.8959f },
        };
        for (int i = 0; i < 3; i++)
        {
            float tip[2] = { ref[i].x, ref[i].z };
            float deg[3];
            CHECK(Kin_IkNormal(tip, -90.0f, deg));
            CHECK_NEAR(deg[0], ref[i].t1 + 90.0, 0.05);
            CHECK_NEAR(deg[1], ref[i].t2 + 90.0, 0.05);
            CHECK_NEAR(deg[2], ref[i].t4 + 90.0, 0.05);
            /* 正解回爪心：往返闭合 */
            float cc[2];
            Kin_FkNormal(deg, cc);
            CHECK_NEAR(cc[0], ref[i].x, 0.05);
            CHECK_NEAR(cc[1], ref[i].z, 0.05);
        }
        /* Σ 恒定检查：三点 θ1+θ2+θ4（URDF）都应 = 140.5°（用户量化值） */
        for (int i = 0; i < 3; i++)
        {
            CHECK_NEAR(ref[i].t1 + ref[i].t2 + ref[i].t4, 140.5, 0.01);
        }
    }

}
