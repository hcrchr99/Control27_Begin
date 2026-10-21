/**
 * @file    test_tune.c
 * @brief   测试项 13=TUNE：速度环独立整定台（与开环台 6 分离）
 *
 * 组成：
 *  - 启动自检（防正反馈）：开环 ±15% duty 各 1s，验证编码器双向计数且
 *    "正 duty → 正测速"（×SIGN 语义）——不成立即正反馈自激风险，锁死拒绝
 *    闭环（处置：查共地→对调该路电机线→查 CHx_ENC 映射，修好后复位重跑）；
 *  - 命令通道：UART4（VOFA+ 数据口复用，双向）文本行，VOFA+ 控件
 *    （按钮发 ON\n / 滑条发 R%.1f\n）或任意串口助手（921600）下发：
 *        P<kp> I<ki> D<kd>   设 PID（立即生效，如 P0.005 I0.05 D0）
 *        F<kff>              设前馈增益（如 F0.0033，标定法见 robot_config.h）
 *        R<rpm>              设速度指令（R60 / R-30，钳位 MAX_RPM）
 *        ON / OFF            功率级使能 / 关断
 *        SHOW                回显参数、指令与状态
 *        M                   M/T 探针快照（四通道边沿采样环 vs 闭环 M 法对照，
 *                            mt-speed-pending 板验观察口，锁死态也可用）
 *        T                   [STAT] 基线流开关（默认开；VOFA+ 对照用，
 *                            纯文本调试嫌它刷屏就关掉）
 *  - 数据回传：1kHz JustFloat 四通道 {ref, 实测rpm, duty, Iout}（bsp_vofa）；
 *  - 自检通过后 ref=0 + Enable：电机保持静止待命，发 R 即转。
 *
 * 整定流程：SHOW 核对 → 拖 R 滑条看阶跃波形 → 改 P/I 迭代 → 终值抄回
 * robot_config.h（运行时改动不做持久化，配置文件才是唯一真值）。
 * 前馈标定（一次做完）：R 给定 → 稳定后 STAT/SHOW 读 duty → F<duty÷ref>
 * 正反各测一点取均值 → 终值抄回 robot_config.h 的 ROBOT_MOTOR_SPEED_KFF。
 *
 * 单电机测试态（同台 6）：只整定 CH4；恢复多路时把 TUNE_CH 换成循环全通道。
 *
 * 历史：曾内置继电反馈自整定（Åström-Hägglund，AUTO 命令，能自动算出参数
 * 初值），板上实测效果不及手调，2026-10-09 应用户要求移除。
 */
#include "test_bench.h"
#include "cmsis_os.h"
#include "motor.h"
#include "bsp_encoder.h"
#include "bsp_sys.h"
#include "bsp_log.h"
#include "bsp_vofa.h"
#include "robot_config.h"
#include "speed_est.h"
#include <math.h>
#include <stdlib.h>

#define TUNE_CH         MOTOR_CH4
#define TUNE_ENC        ROBOT_MOTOR_CH4_ENC     /* 与 TUNE_CH 同轴编码器 */
#define TUNE_DUTY_PROBE 0.15f   /* 自检开环推力：增益≈200rpm/duty 下 ≈30rpm */
#define TUNE_MIN_COUNTS 100     /* 1s 窗最小有效计数（30rpm 应有 ≈470） */

/* 整定台运行态 */
static bool s_armed;        /* 自检通过、闭环已就绪（1kHz 循环在跑） */
static bool s_enabled;      /* 功率级当前态（记账，STBY 实际态） */
static float s_ref;         /* 当前速度指令（rpm，含符号） */
static bool s_stat_en = true;   /* [STAT] 100ms 基线流开关（T 命令切换） */

/* 0.1 定点打印 rpm（newlib-nano 无 %f；+12.3 → "+12.3"） */
static void Log_RpmX10(float rpm)
{
    long x10 = (long)(rpm * 10.0f);
    bool neg = (x10 < 0);
    unsigned long a = (unsigned long)(neg ? -x10 : x10);
    Log_Printf("%c%lu.%lu", neg ? '-' : '+', a / 10u, a % 10u);
}

static void PrintStatus(void);

/* -------------------- M/T 探针（板验观察口，只读） --------------------
 * ⚠ S-C 起速度环是采样环的常驻消费者（1kHz TakeNew），探针在闭环态让位：
 * M 命令改为直接显示闭环四路 M/T 读数（与整定/VOFA 同源）。
 * 锁死态（速度环不跑、采样环无主）探针接管：1kHz 喂 SpeedEst，手拨轮子
 * 即可分辨"电机坏还是编码器坏"，探针不陪葬。
 * 判据（bsp_encoder.h 脚位一节）：每边沿恰 2 计数，单向匀速转动时窗内
 * dcnt = 2×n（打印口径 200/100）；来回拨 dcnt 是净值、比值小于 2 属正常。
 * 每输出圈 1248 沿 : 2496 计数。CH3 抓 PA7（B 相），行为应与其他通道一致。
 * 2026-10-21 首轮板验教训：拨完再按 M 已过静止超时、读数归零造成困惑 →
 * MT 改峰值保持；快拨击穿锁死态 100Hz 喂的 16 格上限 → 锁死态提到 1kHz。 */
static SpeedEst_t s_mt_est[4];
static SpeedEstDrip_t s_mt_drip[4];     /* 滴喂历史（锁死态 1kHz 每拍只来一两滴） */
static Encoder_Sample_t s_mt_buf[ENC_RING_DEPTH];
static int32_t s_mt_last_cnt[4];    /* 探针自己的计数衔接点（与 batch 首样本差分） */
static uint32_t s_mt_n[4];          /* 自上次 M 打印以来的边沿总数 */
static int32_t s_mt_dcnt[4];        /* 同窗净计数（单向匀速时 = 2×n） */
static float s_mt_peak[4];          /* 同窗 |读数| 峰值（带符号），拨完再按 M 也能看到 */
static uint32_t s_mt_drops[4];      /* 掉队累计次数（1kHz 喂不应出现） */
static uint16_t s_mt_last_head[4];  /* 上次 M 时的边沿产出快照（Δhead=产出量） */

static void MtFeed(void)
{
    /* Bsp 的 Encoder_Sample_t 与算法层 SpeedEst_Sample_t 各持一份（分层禁互
     * include），消费侧逐字段搬运——S-C 的速度环也照此办理 */
    SpeedEst_Sample_t se[ENC_RING_DEPTH];
    for (uint8_t i = 0u; i < 4u; i++)
    {
        bool dropped = false;
        uint8_t n = Encoder_TakeNew((uint8_t)(ENC_CH1 + i), s_mt_buf,
                                    (uint8_t)ENC_RING_DEPTH, &dropped);
        if (dropped)
        {
            s_mt_drops[i]++;
        }
        for (uint8_t j = 0u; j < n; j++)
        {
            se[j].cnt = s_mt_buf[j].cnt;
            se[j].us = s_mt_buf[j].us;
            s_mt_dcnt[i] += se[j].cnt - s_mt_last_cnt[i];
            s_mt_last_cnt[i] = se[j].cnt;
        }
        s_mt_n[i] += n;
        (void)SpeedEstDrip_Update(&s_mt_drip[i], &s_mt_est[i], se, n, Bsp_GetUs());
        float v = s_mt_est[i].out;
        if (fabsf(v) > fabsf(s_mt_peak[i]))
        {
            s_mt_peak[i] = v;
        }
    }
}

static void MtPrint(void)
{
    if (s_armed)
    {
        /* 闭环态：探针已让位采样环，四路读数直接取闭环 M/T（同源可信）。
         * Δhead = 边沿产出（EXTI 真发了几次，bisect 关键量）；drops = 消费
         * 侧掉队。读数归零时：Δhead 同停=编码器/EXTI 静默（查线）；没转的
         * 通道 Δhead 疯涨=噪声假边沿风暴；Δhead 正常而读数 0=软件问题 */
        Log_Printf("[MT] CH   闭环M/T    dhead   drops\r\n");
        for (uint8_t i = 0u; i < 4u; i++)
        {
            uint16_t head = Encoder_Head((uint8_t)(ENC_CH1 + i));
            uint16_t dhead = (uint16_t)(head - s_mt_last_head[i]);
            s_mt_last_head[i] = head;
            Log_Printf("[MT] CH%u ", (unsigned)(i + 1u));
            Log_RpmX10(Motor_GetSpeedRpm((MotorCh_t)i));
            Log_Printf("  %5u  %lu\r\n",
                       (unsigned)dhead, (unsigned long)Motor_MtDrops((MotorCh_t)i));
        }
        Log_Printf("[MT] ref=");
        Log_RpmX10(s_ref);
        Log_Printf("\r\n");
        return;
    }

    /* 锁死态：手拨判编码器侧（峰值保持表） */
    Log_Printf("[MT] CH   峰值M/T     n    dcnt  dcnt/n  drop次\r\n");
    for (uint8_t i = 0u; i < 4u; i++)
    {
        Log_Printf("[MT] CH%u ", (unsigned)(i + 1u));
        /* M/T 维持 counts 语义，打印时乘 SIGN 对齐闭环读数口径 */
        Log_RpmX10(s_mt_peak[i] * (float)ROBOT_MOTOR_SIGN);
        if (s_mt_n[i] > 0u)
        {
            Log_Printf("  %lu  %ld  %ld/100  %lu\r\n",
                       (unsigned long)s_mt_n[i], (long)s_mt_dcnt[i],
                       (long)((int64_t)s_mt_dcnt[i] * 100 / (int64_t)s_mt_n[i]),
                       (unsigned long)s_mt_drops[i]);
        }
        else
        {
            Log_Printf("  0     0     --      %lu\r\n", (unsigned long)s_mt_drops[i]);
        }
        s_mt_n[i] = 0u;
        s_mt_dcnt[i] = 0;
        s_mt_peak[i] = 0.0f;
        s_mt_drops[i] = 0u;
    }
    Log_Printf("[MT] 判据：单向匀速拨 dcnt/n=200/100；来回拨 dcnt 是净值，比值<2 正常\r\n");
}

/* -------------------- 启动自检：负反馈确认 -------------------- */

/* 开环推一个方向 1s，返回 SIGN 修正后的测速语义计数（正=正 duty 方向） */
static int32_t ProbeDirection(float duty)
{
    (void)Encoder_Read(TUNE_ENC);               /* 清旧快照，只看本窗 */
    Motor_Enable();
    Motor_SetDuty(TUNE_CH, duty);
    osDelay(1000u);
    Motor_SetDuty(TUNE_CH, 0.0f);
    return Encoder_Read(TUNE_ENC) * ROBOT_MOTOR_SIGN;
}

static bool SelfTest(void)
{
    Log_Printf("[TUNE] 自检：开环 +%ld%% 1s → -%ld%% 1s（编码器双向+符号判据）\r\n",
               (long)(TUNE_DUTY_PROBE * 100.0f), (long)(TUNE_DUTY_PROBE * 100.0f));

    int32_t inc_pos = ProbeDirection(+TUNE_DUTY_PROBE);
    int32_t inc_neg = ProbeDirection(-TUNE_DUTY_PROBE);
    Motor_Disable();

    bool pos_ok = (inc_pos >= TUNE_MIN_COUNTS);
    bool neg_ok = (inc_neg <= -TUNE_MIN_COUNTS);
    Log_Printf("[TUNE] 自检结果：+%ld%%→%+ld counts(%s)  -%ld%%→%+ld counts(%s)\r\n",
               (long)(TUNE_DUTY_PROBE * 100.0f), (long)inc_pos, pos_ok ? "OK" : "FAIL",
               (long)(TUNE_DUTY_PROBE * 100.0f), (long)inc_neg, neg_ok ? "OK" : "FAIL");

    if (pos_ok && neg_ok)
    {
        Log_Printf("[TUNE] 负反馈确认，进入整定态（ref=0 已 Enable，发 R 即转）\r\n");
        return true;
    }
    Log_Printf("[TUNE] ⚠ 正反馈风险/编码器异常：锁死闭环。排查：共地→对调该路电机线"
               "→查 CHx_ENC 映射，修好后复位重跑\r\n");
    return false;
}

/* -------------------- 命令解析 -------------------- */

/* "[-]digits[.digits]" 保守解析：跳过命令字母后的非数字前缀——
 * VOFA+ 控件常发 "R:15" 冒号分隔，手敲 "R15" 也兼容 */
static float ParseFloat(const char *s)
{
    while (*s != '\0' && *s != '-' && (*s < '0' || *s > '9'))
    {
        s++;
    }
    return strtof(s, NULL);
}

static void PrintStatus(void)
{
    float kp, ki, kd, kff;
    Motor_GetTune(&kp, &ki, &kd, &kff);
    Log_Printf("[TUNE] Kp=%ld/10000 Ki=%ld/10000 Kd=%ld/10000 Kff=%ld/10000 ref=",
               (long)(kp * 10000.0f), (long)(ki * 10000.0f),
               (long)(kd * 10000.0f), (long)(kff * 10000.0f));
    Log_RpmX10(s_ref);
    Log_Printf(" rpm 功率级=%s 闭环=%s\r\n",
               s_enabled ? "ON" : "OFF", s_armed ? "ARMED" : "LOCKED");
}

static void HandleCommand(const char *line)
{
    switch (line[0])
    {
    case 'M': case 'm':
        MtPrint();
        break;
    case 'P': case 'p':
    case 'I': case 'i':
    case 'D': case 'd':
    {
        float kp, ki, kd, kff;
        Motor_GetTune(&kp, &ki, &kd, &kff);
        float v = ParseFloat(line + 1);
        if (line[0] == 'P' || line[0] == 'p') { kp = v; }
        if (line[0] == 'I' || line[0] == 'i') { ki = v; }
        if (line[0] == 'D' || line[0] == 'd') { kd = v; }
        Motor_SetTune(kp, ki, kd, kff);
        Log_Printf("[TUNE] PID 更新\r\n");
        PrintStatus();
        break;
    }
    case 'F': case 'f':
    {
        float kp, ki, kd, kff;
        Motor_GetTune(&kp, &ki, &kd, &kff);
        kff = ParseFloat(line + 1);
        Motor_SetTune(kp, ki, kd, kff);
        Log_Printf("[TUNE] Kff 更新（稳态 duty÷ref 即标定值）\r\n");
        PrintStatus();
        break;
    }
    case 'R': case 'r':
        s_ref = ParseFloat(line + 1);
        Motor_SetSpeedRpm(TUNE_CH, s_ref);
        Log_Printf("[TUNE] ref=");
        Log_RpmX10(s_ref);
        Log_Printf(" rpm\r\n");
        break;
    case 'O': case 'o':
        if (line[1] == 'N' || line[1] == 'n')   /* ON 全大小写兼容（W2.3 复盘遗留修复） */
        {
            Motor_Enable();
            s_enabled = true;
            Log_Printf("[TUNE] 功率级 ON\r\n");
        }
        else if ((line[1] == 'F' || line[1] == 'f') &&
                 (line[2] == 'F' || line[2] == 'f'))
        {
            Motor_Disable();            /* 同时清 ref/积分态 */
            s_ref = 0.0f;
            s_enabled = false;
            Log_Printf("[TUNE] 功率级 OFF（ref/积分已清）\r\n");
        }
        break;
    case 'T': case 't':
        s_stat_en = !s_stat_en;
        Log_Printf("[TUNE] [STAT] 流=%s\r\n", s_stat_en ? "ON" : "OFF");
        break;
    case 'S': case 's':
        PrintStatus();
        break;
    default:
        Log_Printf("[TUNE] 未知命令 '%s'（可用：P/I/D/F/R/ON/OFF/SHOW/M/T）\r\n", line);
        break;
    }
}

/* -------------------- 测试台入口 -------------------- */

void Test_Tune_Init(void)
{
    Encoder_InitAll();
    Motor_Init();       /* STBY 保持低 + 速度环装参清态 */
    Bsp_Vofa_Init();    /* UART4 @921600 + 命令接收入口 */

    for (uint8_t i = 0u; i < 4u; i++)
    {
        SpeedEst_Config_t c = { (float)ROBOT_ENC_PPR, 4000u, 30000u, 0.0f };
        SpeedEst_Init(&s_mt_est[i], &c);
        SpeedEstDrip_Init(&s_mt_drip[i]);
        s_mt_last_cnt[i] = 0;
        s_mt_n[i] = 0u;
        s_mt_dcnt[i] = 0;
        s_mt_peak[i] = 0.0f;
        s_mt_drops[i] = 0u;
        s_mt_last_head[i] = 0u;
    }

    float kp, ki, kd, kff;
    Motor_GetTune(&kp, &ki, &kd, &kff);
    Log_Printf("[TUNE] 13=TUNE 速度环整定台：Kp=%ld/10000 Ki=%ld/10000 Kd=%ld/10000 "
               "Kff=%ld/10000 MT窗=%lums 超时=%lums PPR=%ld\r\n",
               (long)(kp * 10000.0f), (long)(ki * 10000.0f),
               (long)(kd * 10000.0f), (long)(kff * 10000.0f),
               (unsigned long)(ROBOT_MT_WIN_MIN_US / 1000u),
               (unsigned long)(ROBOT_MT_STALE_US / 1000u),
               (long)ROBOT_ENC_PPR);
    Log_Printf("[TUNE] VOFA+ JustFloat 4ch @UART4/921600：{ref, 实测, duty, Iout}\r\n");
    Log_Printf("[TUNE] 命令（同口发文本行）：P<kp> I<ki> D<kd> F<kff> R<rpm> ON OFF SHOW\r\n");

    s_armed = SelfTest();
    if (s_armed)
    {
        (void)Encoder_Read(TUNE_ENC);   /* 清自检残速，闭环从干净测速起步 */
        Motor_Enable();                 /* ref=0 静止待命 */
        s_enabled = true;
        s_ref = 0.0f;
    }
    /* 波特率自证：BRR 反算实际值（921600@42MHz → BRR=45 → 933333，+1.3% 合法） */
    Log_Printf("[TUNE] UART4 实测波特率=%lu（目标 921600，±2%% 内合法）\r\n",
               (unsigned long)Bsp_Vofa_GetBaud());
    PrintStatus();
}

void Test_Tune_Poll(void)
{
    /* 命令优先（整定操作不等节拍） */
    char line[ROBOT_VOFA_RX_LINE];
    while (Bsp_Vofa_ReadLine(line, (uint8_t)sizeof line))
    {
        if (s_armed)
        {
            HandleCommand(line);
        }
        else if (line[0] == 'M' || line[0] == 'm')
        {
            MtPrint();      /* 探针在锁死态也活着：手拨即可验编码器侧 */
        }
        else
        {
            Log_Printf("[TUNE] 闭环已锁死，忽略命令 '%s'\r\n", line);
        }
    }

    if (!s_armed)
    {
        /* 锁死态：速度环不跑、采样环无主，探针接管按 1kHz 喂（16 格 @1kHz
         * = 持续 1.6kHz 边沿上限，快拨一下 ~1 圈/秒 = 1248 边沿/秒不溢出） */
        for (uint8_t i = 0u; i < 10u; i++)
        {
            MtFeed();
            osDelay(1u);
        }
        return;             /* 锁死态：只应答命令，不出力 */
    }

    /* 基线状态流（100ms 一行，CSV 数值行，T 命令可关）：VOFA+ 用第二个连接
     * （COM16@115200，FireWater 协议）画它——与 COM14 JustFloat 同屏对照，
     * 谷只在 COM14 出现 = 链路/解析假象；两边同步 = 板上真值 */
    if (s_stat_en)
    {
        static uint32_t s_stat_last;
        uint32_t now = Bsp_GetMs();
        if ((now - s_stat_last) >= 100u)
        {
            s_stat_last = now;
            Log_Printf("[STAT],%ld,%ld,%ld,%ld\r\n",
                       (long)(Motor_GetSpeedRpm(TUNE_CH) * 10.0f),
                       (long)(Motor_GetDuty(TUNE_CH) * 1000.0f),
                       (long)(Motor_GetIout(TUNE_CH) * 1000.0f),
                       (long)(s_ref * 10.0f));
        }
    }

    /* 1kHz 闭环 × 10 拍（补齐测试台 10ms 轮询），每拍同步发 VOFA 流。
     * ⚠ 此环里探针不再喂采样环：S-C 起速度环是采样环的常驻消费者 */
    for (uint8_t i = 0u; i < 10u; i++)
    {
        Motor_SpeedLoopUpdate();
        float vofa[4] = {
            s_ref,
            Motor_GetSpeedRpm(TUNE_CH),
            Motor_GetDuty(TUNE_CH),
            Motor_GetIout(TUNE_CH),
        };
        (void)Bsp_Vofa_SendFloats(vofa, 4u);
        osDelay(1u);
    }
}
