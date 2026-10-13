/**
 * @file    test_servo.c
 * @brief   S5 测试项 7=SERVO：舵机标定台（手动脉宽逐 id 找行程端点）
 *          + 自动验收序列（中位→角度扫描→SG90/PM10S 卸力手掰）
 *
 * 验收标准（Modules 层开发规划 §3.2）：上电缓至中位无猛冲；0→90→180° 扫描
 * 平滑；SG90 / PM10S Release 后手掰均无保持力矩（2026-10-05 修正：PM10S
 * 可卸力，非 fail-hold）
 *
 * W2.4 第 0 项扩展（2026-10-10）：原自动序列保留、命令 G 触发；默认进入
 * 标定待命态，经 UART4（921600，与 13=TUNE 同口同法，VOFA+ 控件或串口
 * 助手发文本行）收命令：
 *   W<id> <us>   直发脉宽（µs），绕过角度换算与软限位——标定主力命令，
 *                按组钳帧长上限（250Hz 组 ≤4000 / 50Hz 组 ≤20000）
 *   Q<id>        卸力（停脉冲，验证手掰无保持力矩）
 *   G            跑自动验收序列（扫描 + 卸力），2s/步
 *   S            重打帮助与标定流程
 * 任意命令使自动序列切回手动待命态。
 *
 * 标定流程（结果回填 robot_config.h，不留"待实测"过夜）：
 *   1. 上电等 0.3s 中位稳定窗过后，W<id> 1500 从中位起步；
 *   2. 向小步进（每次 -50µs）：舵机顶到机械端点会嗡鸣/抖动，回退 50µs
 *      记下脉宽 = ROBOT_SERVO_PULSE_MIN_US_LIST[id]；同法向大记 MAX；
 *   3. 量角器记两端机械限位角 → 收紧 ROBOT_SERVO_LIMIT_*_DEG_LIST
 *      （软限位应比机械限位再收 5~10°）；
 *   4. PM10S 组（ID1/ID2）示波器看 PE5/PE6：250Hz 帧率 + 脉宽与命令一致。
 */
#include "test_bench.h"
#include "servo.h"
#include "bsp_sys.h"
#include "bsp_log.h"
#include "bsp_pwm.h"
#include "bsp_vofa.h"
#include "robot_config.h"
#include <stdlib.h>

/* id → bsp_pwm 脉宽通道（与 servo.c 同源 robot_config.h 的配置宏） */
static const uint8_t s_ch[SERVO_COUNT] = ROBOT_SERVO_PWM_CH_LIST;

static bool s_auto;         /* true = 自动验收序列进行中 */
static uint32_t s_auto_last;
static uint8_t  s_auto_step;    /* 0..4 = 扫描，5 = SG90 Release，6 = PM10S Release */

static void Help(void);

/* -------------------- 手动命令 -------------------- */

/* 跳到首个数字/负号处（VOFA+ 控件常发 "W1:1500" 冒号分隔，一并兼容） */
static const char *SkipNonDigit(const char *s)
{
    while (*s != '\0' && *s != '-' && (*s < '0' || *s > '9'))
    {
        s++;
    }
    return s;
}

static void HandleCommand(const char *line)
{
    switch (line[0])
    {
    case 'W': case 'w':
    {
        const char *p = SkipNonDigit(line + 1);
        long id = strtol(p, (char **)&p, 10);
        long us = strtol(SkipNonDigit(p), NULL, 10);
        if (id < 1 || id > SERVO_COUNT || us <= 0)
        {
            Log_Printf("[T-SERVO] W 参数非法（id=1..%d, us>0），如 W1 1500\r\n",
                       (int)SERVO_COUNT);
            break;
        }
        Pwm_SetPulseUs(s_ch[id - 1u], (uint32_t)us);
        const char *grp = (s_ch[id - 1u] <= PWM_250HZ_CH2) ? "TIM9 250Hz" : "TIM5 50Hz";
        Log_Printf("[T-SERVO] ID%ld → %ldµs（ch%u %s）\r\n",
                   id, us, s_ch[id - 1u], grp);
        break;
    }
    case 'Q': case 'q':
    {
        const char *p = SkipNonDigit(line + 1);
        long id = strtol(p, NULL, 10);
        if (id < 1 || id > SERVO_COUNT)
        {
            Log_Printf("[T-SERVO] Q 参数非法（id=1..%d）\r\n", (int)SERVO_COUNT);
            break;
        }
        Pwm_Release(s_ch[id - 1u]);
        Log_Printf("[T-SERVO] ID%ld 已卸力（停脉冲），手掰应无保持力矩\r\n", id);
        break;
    }
    case 'G': case 'g':
        s_auto = true;
        s_auto_step = 0u;
        s_auto_last = 0u;
        Log_Printf("[T-SERVO] 自动验收序列启动（2s/步，任意命令切回手动）\r\n");
        break;
    case 'S': case 's':
        Help();
        break;
    default:
        Log_Printf("[T-SERVO] 未知命令 '%s'\r\n", line);
        Help();
        break;
    }
}

/* -------------------- 自动验收序列（S5 原样保留） -------------------- */

static void AutoStep(void)
{
    uint32_t now = Bsp_GetMs();
    if ((now - s_auto_last) < 2000u)
    {
        return;
    }
    s_auto_last = now;

    static const float s_sweep[] = { 0.0f, 90.0f, 180.0f, 90.0f, 0.0f };
    const uint8_t n = (uint8_t)(sizeof(s_sweep) / sizeof(s_sweep[0]));

    if (s_auto_step < n)
    {
        float deg = s_sweep[s_auto_step];
        for (uint8_t id = 0u; id < SERVO_COUNT; id++)
        {
            Servo_SetAngle((ServoId_t)id, deg);
        }
        Log_Printf("[T-SERVO] 五路同步 %u°（限位钳位后 5..175）\r\n", (unsigned)deg);
    }
    else if (s_auto_step == n)
    {
        bool wrist = Servo_Release(SERVO_ID3);
        bool claw  = Servo_Release(SERVO_ID4);
        bool wrist_roll = Servo_Release(SERVO_ID5);
        Log_Printf("[T-SERVO] SG90 已 Release（真卸力 手腕=%d 手腕旋转=%d 爪子=%d，应=1）："
                   "手掰应无保持力矩\r\n", wrist, wrist_roll, claw);
    }
    else
    {
        bool arm  = Servo_Release(SERVO_ID1);
        bool fore = Servo_Release(SERVO_ID2);
        Log_Printf("[T-SERVO] PM10S 已 Release（卸力 大臂=%d 小臂=%d，应=1）："
                   "手掰应无保持力矩\r\n", arm, fore);
    }
    s_auto_step = (uint8_t)((s_auto_step + 1u) % (n + 2u));
}

static void Help(void)
{
    Log_Printf("[T-SERVO] 7=SERVO 标定台（命令走 UART4 921600 文本行）：\r\n");
    Log_Printf("[T-SERVO]   W<id> <us>  手动脉宽（如 W1 1500；绕过软限位）\r\n");
    Log_Printf("[T-SERVO]   Q<id>       卸力   G 自动验收序列   S 重打本帮助\r\n");
    Log_Printf("[T-SERVO] 标定：W<id> 1500 起步 → ±50µs 步进找两端嗡鸣点回退 50 →\r\n");
    Log_Printf("[T-SERVO] 记 PULSE_MIN/MAX_US_LIST；量角记 LIMIT；PM10S 示波器看 PE5/PE6@250Hz\r\n");
}

/* -------------------- 测试台入口 -------------------- */

void Test_Servo_Init(void)
{
    Servo_InitAll();
    Bsp_Vofa_Init();    /* UART4 @921600 + 命令接收入口（标定命令通道） */

    s_auto = false;     /* 默认标定待命态，等命令（G 进自动序列） */
    Log_Printf("[T-SERVO] 7=SERVO 标定台就绪（中位稳定窗 300ms 过后可控）\r\n");
    Help();
}

void Test_Servo_Poll(void)
{
    /* 命令优先；任意命令把自动序列切回手动 */
    char line[ROBOT_VOFA_RX_LINE];
    while (Bsp_Vofa_ReadLine(line, (uint8_t)sizeof line))
    {
        s_auto = false;
        HandleCommand(line);
    }

    if (s_auto)
    {
        AutoStep();
    }
}
