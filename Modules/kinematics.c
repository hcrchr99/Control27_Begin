/**
 * @file    kinematics.c
 * @brief   机械臂平面运动学纯函数实现（设计说明见 kinematics.h）
 *
 * 几何口径：真实链是 肩(Y轴)→肘(Y轴)→腕转(斜轴,过 R0 点)→腕俯仰(Y轴)→
 * 爪(固定安装向量 G)。腕转轴斜置且轴心不在臂面，爪旋转 φ 一转，腕俯仰
 * 轴和爪尖都被甩出臂面。本文件把"旋转后的腕+爪"折算成一根随 φ 变化的
 * 等效连杆 D（肘→爪尖，肘坐标系面内向量），推导要点：
 *
 *   绕面内单位轴 a=(ax,az) 转 φ 后，向量 w=(wx,wy,wz) 的面内分量
 *     [Ra(φ)w]_xz = cosφ·w_xz + sinφ·wy·perp(a) + (1−cosφ)(a·w)·a
 *   其中 perp(a)=(−az,ax)（a 恒无 y 分量，a·w 只看面内点积）。
 *   腕俯仰 Ry(q3) 只把 G 的面内分量转 −q3，y 分量不变。
 *
 *   于是 爪尖(面内) = 肩 + Rot(−q1)·U + Rot(−(q1+q2))·D(φ,q3)，
 *   与平面 2R 完全同构：φ、q3 固定时 D 是常向量，闭式可解、不迭代。
 *   （Rot(−q) 指绕 Y 轴 q 在面内等效的二维旋转；仰角 el = −q，+为抬头。）
 */
#include "kinematics.h"
#include "robot_config.h"
#include <math.h>

#define KIN_PI_f    3.14159265358979f
#define KIN_D2R     (KIN_PI_f / 180.0f)

/* 肘弯折方向：+1 = 与建模姿态同向的单弯折（§3.9⑤：不双解、拒绝比绕行）。
 * 建模姿态下两连杆夹角 ≈ 0.7°（小臂基准仰角 30.4° 略高于大臂 29.7°），
 * 属正弯向；实机若装配反向，只改这里重标 */
#define KIN_ELBOW_BEND  (+1.0f)

/* 软限位（舵机度）借 servo 占位表前三路——同一个 0..180° 坐标系单一来源；
 * 仰角域换算 el = 90 − servo_deg，实机收紧限位后本模块自动跟进 */
static const float kServoLimitMin[ROBOT_CMD_JOINT_COUNT] = ROBOT_SERVO_LIMIT_MIN_DEG_LIST;
static const float kServoLimitMax[ROBOT_CMD_JOINT_COUNT] = ROBOT_SERVO_LIMIT_MAX_DEG_LIST;

/* ---- 常量几何（robot_config 单一来源，这里只做一次合成） ---- */

static float upper_len(void)    /* 大臂 |U| */
{
    float x = ROBOT_ARM_UPPER_X_MM, z = ROBOT_ARM_UPPER_Z_MM;
    return sqrtf(x * x + z * z);
}

static float upper_base_elev(void)  /* 大臂基准仰角 β1（建模姿态抬头角，度） */
{
    return atan2f(ROBOT_ARM_UPPER_Z_MM, ROBOT_ARM_UPPER_X_MM) / KIN_D2R;
}

static float fore_len(void)                 /* 小臂 |P|（肘→腕俯仰轴） */
{
    float x = ROBOT_ARM_ROLLJ_X_MM + ROBOT_ARM_WRIST_X_MM;
    float z = ROBOT_ARM_ROLLJ_Z_MM + ROBOT_ARM_WRIST_Z_MM;
    return sqrtf(x * x + z * z);
}

static float fore_elev_deg(void)            /* 小臂基准仰角，度 */
{
    float x = ROBOT_ARM_ROLLJ_X_MM + ROBOT_ARM_WRIST_X_MM;
    float z = ROBOT_ARM_ROLLJ_Z_MM + ROBOT_ARM_WRIST_Z_MM;
    return atan2f(z, x) / KIN_D2R;
}

static void tool_link(float *len, float *base_elev) /* 爪安装向量 G（面内） */
{
    float x = ROBOT_ARM_TOOL_X_MM, z = ROBOT_ARM_TOOL_Z_MM;
    *len = sqrtf(x * x + z * z);
    *base_elev = atan2f(z, x) / KIN_D2R;
}

static float servo_to_el(float servo_deg)   /* 舵机角 → 仰角（+抬头） */
{
    return 90.0f - servo_deg;
}

static float el_to_servo(float el_deg)      /* 仰角 → 舵机角 */
{
    return 90.0f - el_deg;
}

static bool el_in_limits(float el_deg, int joint)
{
    /* 舵机 [min,max] ↔ 仰角 [90−max, 90−min]，方向相反 */
    return (el_deg >= 90.0f - kServoLimitMax[joint]) &&
           (el_deg <= 90.0f - kServoLimitMin[joint]);
}

/* ---- 等效连杆 D(φ, 腕仰角)：肘→爪尖（肘坐标系面内）+ 爪尖横向偏移 ---- */

static void eff_link(float roll_rad, float el3_deg,
                     float *bx, float *bz, float *tip_y)
{
    /* 腕俯仰 Ry(q3) 对面内向量的作用 = 二维旋转 −q3 = +el3（q3=−el3） */
    float a3 = el3_deg * KIN_D2R;
    float ca3 = cosf(a3), sa3 = sinf(a3);
    float wx = ROBOT_ARM_WRIST_X_MM
             + ROBOT_ARM_TOOL_X_MM * ca3 - ROBOT_ARM_TOOL_Z_MM * sa3;
    float wz = ROBOT_ARM_WRIST_Z_MM
             + ROBOT_ARM_TOOL_X_MM * sa3 + ROBOT_ARM_TOOL_Z_MM * ca3;
    float wy = ROBOT_ARM_WRIST_Y_MM + ROBOT_ARM_TOOL_Y_MM;  /* Ry 不改 y，恒定 */

    float cr = cosf(roll_rad), sr = sinf(roll_rad);
    float ax = ROBOT_ARM_ROLL_AX_X, az = ROBOT_ARM_ROLL_AX_Z;
    float px = -az, pz = ax;            /* perp(a) */
    float adotw = ax * wx + az * wz;    /* a 无 y 分量，点积只看面内 */

    /* [Ra(φ)w]_xz = cosφ·w + sinφ·wy·perp(a) + (1−cosφ)(a·w)a */
    float tx = cr * wx + sr * wy * px + (1.0f - cr) * adotw * ax;
    float tz = cr * wz + sr * wy * pz + (1.0f - cr) * adotw * az;

    *bx = ROBOT_ARM_ROLLJ_X_MM + tx;
    *bz = ROBOT_ARM_ROLLJ_Z_MM + tz;
    /* 横向：y 分量 = R0y + wy·cosφ + (a×w)_y·sinφ */
    *tip_y = ROBOT_ARM_ROLLJ_Y_MM + wy * cr + (az * wx - ax * wz) * sr;
}

/* ---- 平面 2R 逆解核：肩原点 + 两连杆（长度/基准仰角）→ 前两关节仰角 ---- */

static bool solve2r(float tx_mm, float tz_mm, float len_a, float len_b,
                    float base_elev_a, float base_elev_b, float bend_sign,
                    float *el1, float *el2)
{
    float dx = tx_mm - ROBOT_ARM_SHOULDER_X_MM;
    float dz = tz_mm - ROBOT_ARM_SHOULDER_Z_MM;
    float r2 = dx * dx + dz * dz;
    /* 余弦定理：Δ = 两连杆绝对仰角差；|cosΔ|→1 即伸直/折叠奇异。
     * 三角计算用弧度，出口统一转度（仰角域全程为度） */
    float c = (r2 - len_a * len_a - len_b * len_b) / (2.0f * len_a * len_b);
    if (c > 1.0f - ROBOT_ARM_SING_MARGIN || c < -(1.0f - ROBOT_ARM_SING_MARGIN))
    {
        return false;
    }
    float bend = bend_sign * acosf(c);
    /* 第一关节绝对仰角 = 目标方向 − 半张角修正（标准 2R 公式） */
    float th1 = atan2f(dz, dx)
              - atan2f(len_b * sinf(bend), len_a + len_b * cosf(bend));
    *el1 = (th1 - base_elev_a * KIN_D2R) / KIN_D2R;
    /* 第二关节：absB = Θ1 + Δ，且 absB = base_elev_b + el1 + el2 */
    *el2 = bend / KIN_D2R + base_elev_a - base_elev_b;
    return true;
}

/* ---- 对外 API ---- */

void Kin_Fk(const float deg[3], float roll_deg, Kin_Pose_t *out)
{
    float el1 = servo_to_el(deg[0]);
    float el2 = servo_to_el(deg[1]);
    float el3 = servo_to_el(deg[2]);

    float bx, bz, tip_y;
    eff_link(roll_deg * KIN_D2R, el3, &bx, &bz, &tip_y);
    float len_b = sqrtf(bx * bx + bz * bz);
    float base_b = atan2f(bz, bx);

    /* 大臂绝对仰角 = β1 + el1；等效连杆绝对仰角 = 基准 + (el1+el2) */
    float a1 = (upper_base_elev() + el1) * KIN_D2R;
    float a2 = (base_b / KIN_D2R + el1 + el2) * KIN_D2R;
    float la = upper_len();

    out->x_mm = ROBOT_ARM_SHOULDER_X_MM + la * cosf(a1) + len_b * cosf(a2);
    out->z_mm = ROBOT_ARM_SHOULDER_Z_MM + la * sinf(a1) + len_b * sinf(a2);
    out->y_mm = tip_y;
    float open[2];
    Kin_OpeningDir(roll_deg, open);
    out->open_x = open[0];
    out->open_z = open[1];
}

void Kin_OpeningDir(float roll_deg, float out[2])
{
    /* 开合方向 = Ra(φ)·ŷ：面内投影 = sinφ·perp(a)，横向分量 = cosφ */
    float s = sinf(roll_deg * KIN_D2R);
    out[0] = s * (-ROBOT_ARM_ROLL_AX_Z);
    out[1] = s * (ROBOT_ARM_ROLL_AX_X);
}

bool Kin_IkTool(const float tip_mm[2], float tool_pitch_deg, float deg_out[3])
{
    /* 先扣掉爪安装向量得腕位置，再对 U+P 做两连杆逆解 */
    float tool_len, tool_elev;
    tool_link(&tool_len, &tool_elev);
    float ps = tool_pitch_deg * KIN_D2R;
    float wx = tip_mm[0] - tool_len * cosf(ps);
    float wz = tip_mm[1] - tool_len * sinf(ps);

    float fore_l = fore_len(), fore_e = fore_elev_deg();
    float el1, el2;
    if (!solve2r(wx, wz, upper_len(), fore_l,
                 upper_base_elev(), fore_e, KIN_ELBOW_BEND, &el1, &el2))
    {
        return false;
    }
    /* 腕补偿：爪安装向量挂腕俯仰系，abs指向 = tool_elev + el1+el2+el3 = 期望值
     * （不经过小臂基准角——G 不随肘系基准仰角转，只随关节角转） */
    float el3 = tool_pitch_deg - tool_elev - el1 - el2;
    if (!el_in_limits(el1, 0) || !el_in_limits(el2, 1) || !el_in_limits(el3, 2))
    {
        return false;
    }
    deg_out[0] = el_to_servo(el1);
    deg_out[1] = el_to_servo(el2);
    deg_out[2] = el_to_servo(el3);
    return true;
}

bool Kin_IkOpen(const float tip_mm[2], float roll_deg, float wrist_deg,
                float deg_out[3])
{
    float el3 = servo_to_el(wrist_deg);
    float bx, bz, tip_y;
    eff_link(roll_deg * KIN_D2R, el3, &bx, &bz, &tip_y);
    (void)tip_y;    /* 横向偏移不由逆解控制，仅 FK/遥测用 */

    float el1, el2;
    if (!solve2r(tip_mm[0], tip_mm[1], upper_len(), sqrtf(bx * bx + bz * bz),
                 upper_base_elev(), atan2f(bz, bx) / KIN_D2R,
                 KIN_ELBOW_BEND, &el1, &el2))
    {
        return false;
    }
    if (!el_in_limits(el1, 0) || !el_in_limits(el2, 1) || !el_in_limits(el3, 2))
    {
        return false;
    }
    deg_out[0] = el_to_servo(el1);
    deg_out[1] = el_to_servo(el2);
    deg_out[2] = wrist_deg;     /* 全程恒定，原样带回凑齐三关节 */
    return true;
}

/* ---- NORMAL 模式（爪板沿法向纯平移；常量=用户网格实测 robot_config）----
 * 仰角域推导：爪板法向仰角 = α(n0) + (el1+el2+el4)（el=−θ，绕 y 正转在
 * (x,z) 面内为顺时针，仰角域等价 CCW 旋转 +el）。给定运动方向 ψ：
 *   Σ_el = ψ − α(n0)
 *   腕俯仰轴心 W = 爪心 − Rot(Σ_el)·R4   （R4 = TCP0 − 腕俯仰轴，常向量）
 *   2R 闭式解 (el1,el2)（连杆=大臂/小臂，与 TOOL 同一套）
 *   el4 = Σ_el − el1 − el2
 * 与用户 claw_traj.py 的数值迭代互为校验（轨迹点对表见 PcTests）。 */

static float normal0_elev(void)             /* α(n0)，度 */
{
    return atan2f(ROBOT_ARM_NORMAL0_Z, ROBOT_ARM_NORMAL0_X) / KIN_D2R;
}

/* R4 = TCP0 − 腕俯仰轴位（腕俯仰轴位 = 肩+大臂+小臂向量，度→仰角域旋转） */
static void r4_vec(float *x, float *z)
{
    *x = ROBOT_ARM_TCP_X_MM - (ROBOT_ARM_SHOULDER_X_MM + ROBOT_ARM_UPPER_X_MM
                               + ROBOT_ARM_ROLLJ_X_MM + ROBOT_ARM_WRIST_X_MM);
    *z = ROBOT_ARM_TCP_Z_MM - (ROBOT_ARM_SHOULDER_Z_MM + ROBOT_ARM_UPPER_Z_MM
                               + ROBOT_ARM_ROLLJ_Z_MM + ROBOT_ARM_WRIST_Z_MM);
}

bool Kin_IkNormal(const float cc_mm[2], float dir_deg, float deg_out[3])
{
    float sigma = dir_deg - normal0_elev();     /* Σ（仰角域），度 */
    float sr = sinf(sigma * KIN_D2R), cr = cosf(sigma * KIN_D2R);
    float r4x, r4z;
    r4_vec(&r4x, &r4z);
    float wx = cc_mm[0] - (r4x * cr - r4z * sr);    /* Rot(Σ)·R4（仰角域 CCW） */
    float wz = cc_mm[1] - (r4x * sr + r4z * cr);

    /* 下压轨迹族在肘负弯折分支（与折叠初始位同族，用户轨迹点实证），
     * 优先 −1；该分支越限/奇异再试 +1。整条直线预扫会按同一优先序取解，
     * 分支中途切换会表现为关节速率尖峰被预扫拒绝，不会静默跳变 */
    float el1, el2;
    if (!solve2r(wx, wz, upper_len(), fore_len(),
                 upper_base_elev(), fore_elev_deg(), -1.0f, &el1, &el2) ||
        !el_in_limits(el1, 0) || !el_in_limits(el2, 1))
    {
        if (!solve2r(wx, wz, upper_len(), fore_len(),
                     upper_base_elev(), fore_elev_deg(), 1.0f, &el1, &el2))
        {
            return false;
        }
    }
    float el4 = sigma - el1 - el2;
    if (!el_in_limits(el1, 0) || !el_in_limits(el2, 1) || !el_in_limits(el4, 2))
    {
        return false;
    }
    deg_out[0] = el_to_servo(el1);
    deg_out[1] = el_to_servo(el2);
    deg_out[2] = el_to_servo(el4);
    return true;
}

void Kin_FkNormal(const float deg[3], float cc_out[2])
{
    float el1 = servo_to_el(deg[0]);
    float el2 = servo_to_el(deg[1]);
    float el4 = servo_to_el(deg[2]);
    float sigma = el1 + el2 + el4;
    float a1 = (upper_base_elev() + el1) * KIN_D2R;
    float a2 = (fore_elev_deg() + el1 + el2) * KIN_D2R;
    float px = ROBOT_ARM_SHOULDER_X_MM
             + upper_len() * cosf(a1) + fore_len() * cosf(a2);
    float pz = ROBOT_ARM_SHOULDER_Z_MM
             + upper_len() * sinf(a1) + fore_len() * sinf(a2);
    float sr = sinf(sigma * KIN_D2R), cr = cosf(sigma * KIN_D2R);
    float r4x, r4z;
    r4_vec(&r4x, &r4z);
    cc_out[0] = px + r4x * cr - r4z * sr;
    cc_out[1] = pz + r4x * sr + r4z * cr;
}

float Kin_NormalDir(const float deg[3])
{
    return normal0_elev() + servo_to_el(deg[0]) + servo_to_el(deg[1])
         + servo_to_el(deg[2]);
}

static bool ik_by_mode(const Kin_Line_t *l, float x_mm, float z_mm, float p,
                       float deg[3])
{
    float tip[2] = { x_mm, z_mm };
    if (l->mode == KIN_MODE_TOOL)
    {
        /* 爪指向沿线从起点姿态线性过渡到目标指向 */
        float pitch = l->tool_pitch0_deg +
                      (l->tool_pitch_deg - l->tool_pitch0_deg) * p;
        return Kin_IkTool(tip, pitch, deg);
    }
    if (l->mode == KIN_MODE_NORMAL)
    {
        return Kin_IkNormal(tip, l->dir_deg, deg);  /* Σ 恒定：朝向严格不变 */
    }
    return Kin_IkOpen(tip, l->roll_deg, l->wrist_deg, deg);
}

bool Kin_LineStart(Kin_Line_t *l, const Kin_LineReq_t *req, uint32_t now_ms)
{
    l->active = false;

    float dx = req->to_x_mm - req->from_x_mm;
    float dz = req->to_z_mm - req->from_z_mm;
    float len = sqrtf(dx * dx + dz * dz);

    /* 全线预扫（§3.9）：每步采一点逆解 + 相邻步关节增量对速率上限校验，
     * 任一点违例整条拒绝——轨迹一步都不动，不会走一半卡死 */
    int n = (int)(len / ROBOT_ARM_LINE_STEP_MM) + 1;
    float seg_mm = len / (float)n;
    float seg_ms = seg_mm / ROBOT_ARM_LINE_SPEED_MMPS * 1000.0f;
    float max_step_deg = ROBOT_APP_JOINT_RATE_DPS * seg_ms * 0.001f;

    float prev[3] = { 0.0f, 0.0f, 0.0f };
    bool prev_valid = false;
    Kin_Line_t probe;   /* 预扫用临时配置（同请求的模式参数，无状态） */
    probe.mode = req->mode;
    probe.tool_pitch_deg = req->tool_pitch_deg;
    probe.tool_pitch0_deg = req->tool_pitch0_deg;
    probe.roll_deg = req->roll_deg;
    probe.wrist_deg = req->wrist_deg;
    probe.dir_deg = req->dir_deg;
    for (int i = 0; i <= n; i++)
    {
        float t = (float)i / (float)n;
        float x = req->from_x_mm + dx * t;
        float z = req->from_z_mm + dz * t;
        float deg[3];
        if (!ik_by_mode(&probe, x, z, t, deg))
        {
            /* 起点采样=当前姿态本身（可能是伸直奇异位，如建模位）——
             * 不必可达，只影响首段速率校验是否生效；中途/终点必须可达 */
            if (i == 0)
            {
                continue;
            }
            return false;
        }
        if (i > 0 && prev_valid)
        {
            for (int j = 0; j < 3; j++)
            {
                if (fabsf(deg[j] - prev[j]) > max_step_deg + 0.01f)
                {
                    return false;   /* 关节峰值速率超上限（邻近奇异前兆） */
                }
            }
        }
        for (int j = 0; j < 3; j++)
        {
            prev[j] = deg[j];
        }
        prev_valid = true;
    }

    l->mode = req->mode;
    l->from_x_mm = req->from_x_mm;
    l->from_z_mm = req->from_z_mm;
    l->to_x_mm = req->to_x_mm;
    l->to_z_mm = req->to_z_mm;
    l->tool_pitch_deg = req->tool_pitch_deg;
    l->tool_pitch0_deg = req->tool_pitch0_deg;
    l->roll_deg = req->roll_deg;
    l->wrist_deg = req->wrist_deg;
    l->dir_deg = req->dir_deg;
    l->t0_ms = now_ms;
    l->total_ms = (uint32_t)(len / ROBOT_ARM_LINE_SPEED_MMPS * 1000.0f + 0.5f);
    if (l->total_ms < 1u)
    {
        l->total_ms = 1u;
    }
    l->active = true;
    return true;
}

void Kin_LineDescend(Kin_LineReq_t *req, const float from_mm[2],
                     float drop_mm, float roll_deg, float wrist_deg)
{
    float u[2];
    Kin_OpeningDir(roll_deg, u);
    req->mode = KIN_MODE_OPEN;
    req->from_x_mm = from_mm[0];
    req->from_z_mm = from_mm[1];
    /* u 指向开合方向（roll=90° 时斜向上），下插取其反向 */
    req->to_x_mm = from_mm[0] - drop_mm * u[0];
    req->to_z_mm = from_mm[1] - drop_mm * u[1];
    req->tool_pitch_deg = 0.0f;
    req->roll_deg = roll_deg;
    req->wrist_deg = wrist_deg;
}

bool Kin_LineStep(Kin_Line_t *l, uint32_t now_ms, float deg_out[3])
{
    if (!l->active)
    {
        return false;
    }
    /* uint32 无符号减法：时基回绕安全（同 grab_ctrl 手法） */
    float p = 1.0f;
    if (now_ms - l->t0_ms < l->total_ms)
    {
        p = (float)(now_ms - l->t0_ms) / (float)l->total_ms;
    }
    float x = l->from_x_mm + (l->to_x_mm - l->from_x_mm) * p;
    float z = l->from_z_mm + (l->to_z_mm - l->from_z_mm) * p;

    float deg[3];
    if (!ik_by_mode(l, x, z, p, deg))
    {
        return false;       /* 本拍取点失败（奇异邻域/数值异常）——是否取消
                             * 由调用方拍板：容错窗内可保持当前角等 p 走出 */
    }
    deg_out[0] = deg[0];
    deg_out[1] = deg[1];
    deg_out[2] = deg[2];
    if (p >= 1.0f)
    {
        l->active = false;  /* 终点拍输出终点角后收摊，调用方查 active 判完成 */
    }
    return true;
}
