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
 *        R<rpm>              设速度指令（R60 / R-30，钳位 MAX_RPM）
 *        ON / OFF            功率级使能 / 关断
 *        SHOW                回显参数、指令与状态
 *  - 数据回传：1kHz JustFloat 四通道 {ref, 实测rpm, duty, Iout}（bsp_vofa）；
 *  - 自检通过后 ref=0 + Enable：电机保持静止待命，发 R 即转。
 *
 * 整定流程：SHOW 核对 → 拖 R 滑条看阶跃波形 → 改 P/I 迭代 → 终值抄回
 * robot_config.h（运行时改动不做持久化，配置文件才是唯一真值）。
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
#include <stdlib.h>

#define TUNE_CH         MOTOR_CH4
#define TUNE_ENC        ROBOT_MOTOR_CH4_ENC     /* 与 TUNE_CH 同轴编码器 */
#define TUNE_DUTY_PROBE 0.15f   /* 自检开环推力：增益≈200rpm/duty 下 ≈30rpm */
#define TUNE_MIN_COUNTS 100     /* 1s 窗最小有效计数（30rpm 应有 ≈470） */

/* 整定台运行态 */
static bool s_armed;        /* 自检通过、闭环已就绪（1kHz 循环在跑） */
static bool s_enabled;      /* 功率级当前态（记账，STBY 实际态） */
static float s_ref;         /* 当前速度指令（rpm，含符号） */

/* 0.1 定点打印 rpm（newlib-nano 无 %f；+12.3 → "+12.3"） */
static void Log_RpmX10(float rpm)
{
    long x10 = (long)(rpm * 10.0f);
    bool neg = (x10 < 0);
    unsigned long a = (unsigned long)(neg ? -x10 : x10);
    Log_Printf("%c%lu.%lu", neg ? '-' : '+', a / 10u, a % 10u);
}

static void PrintStatus(void);

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
    float kp, ki, kd;
    Motor_GetTune(&kp, &ki, &kd);
    Log_Printf("[TUNE] Kp=%ld/10000 Ki=%ld/10000 Kd=%ld/10000 ref=",
               (long)(kp * 10000.0f), (long)(ki * 10000.0f), (long)(kd * 10000.0f));
    Log_RpmX10(s_ref);
    Log_Printf(" rpm 功率级=%s 闭环=%s\r\n",
               s_enabled ? "ON" : "OFF", s_armed ? "ARMED" : "LOCKED");
}

static void HandleCommand(const char *line)
{
    switch (line[0])
    {
    case 'P': case 'p':
    case 'I': case 'i':
    case 'D': case 'd':
    {
        float kp, ki, kd;
        Motor_GetTune(&kp, &ki, &kd);
        float v = ParseFloat(line + 1);
        if (line[0] == 'P' || line[0] == 'p') { kp = v; }
        if (line[0] == 'I' || line[0] == 'i') { ki = v; }
        if (line[0] == 'D' || line[0] == 'd') { kd = v; }
        Motor_SetTune(kp, ki, kd);
        Log_Printf("[TUNE] PID 更新\r\n");
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
    case 'S': case 's':
        PrintStatus();
        break;
    default:
        Log_Printf("[TUNE] 未知命令 '%s'（可用：P/I/D/R/ON/OFF/SHOW）\r\n", line);
        break;
    }
}

/* -------------------- 测试台入口 -------------------- */

void Test_Tune_Init(void)
{
    Encoder_InitAll();
    Motor_Init();       /* STBY 保持低 + 速度环装参清态 */
    Bsp_Vofa_Init();    /* UART4 @921600 + 命令接收入口 */

    float kp, ki, kd;
    Motor_GetTune(&kp, &ki, &kd);
    Log_Printf("[TUNE] 13=TUNE 速度环整定台：Kp=%ld/10000 Ki=%ld/10000 Kd=%ld/10000 "
               "CALC=%ums PPR=%ld\r\n",
               (long)(kp * 10000.0f), (long)(ki * 10000.0f), (long)(kd * 10000.0f),
               (unsigned)ROBOT_MOTOR_SPEED_CALC_MS, (long)ROBOT_ENC_PPR);
    Log_Printf("[TUNE] VOFA+ JustFloat 4ch @UART4/921600：{ref, 实测, duty, Iout}\r\n");
    Log_Printf("[TUNE] 命令（同口发文本行）：P<kp> I<ki> D<kd> R<rpm> ON OFF SHOW\r\n");

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
        else
        {
            Log_Printf("[TUNE] 闭环已锁死，忽略命令 '%s'\r\n", line);
        }
    }

    if (!s_armed)
    {
        return;                             /* 锁死态：只应答命令，不出力 */
    }

    /* 基线状态流（100ms 一行，CSV 数值行）：VOFA+ 用第二个连接
     * （COM16@115200，FireWater 协议）画它——与 COM14 JustFloat 同屏对照，
     * 谷只在 COM14 出现 = 链路/解析假象；两边同步 = 板上真值 */
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

    /* 1kHz 闭环 × 10 拍（补齐测试台 10ms 轮询），每拍同步发 VOFA 流 */
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
