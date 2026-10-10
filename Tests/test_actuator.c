/**
 * @file    test_actuator.c
 * @brief   测试项 10=ACTUATOR：actuator 缓动/堵转/热保护 + alarm 声光联验
 *
 * 实例布局（五路全建，策略按规划 §3.3 保护矩阵）：
 *   ID1 大臂 / ID2 小臂：电流堵转判据（注入 Power_GetJointCurrent），
 *                        热保护禁用（电流是唯一真判据）
 *   ID3 手腕 / ID5 爪旋转：无判据 + 热保护禁用（全程业务性保持，
 *                        防线 = 轻载 + 报警 + 操作手）
 *   ID4 爪开合：无判据 + 热保护使能（触发只报告不强制卸力——
 *                        夹矿时卸不卸由操作手拍板，掉矿代价 < 烧机）
 *
 * 命令（UART4 921600 文本行，与 13=TUNE 同口同法）：
 *   A<id> <deg>   设目标（缓动斜坡，如 A1 90）
 *   R<id>         卸力（Release）
 *   S             打印五实例状态（态/角度/堵转/热 + ID1/ID2 实时电流）
 *   L<n>          注入告警码联验 alarm：0=OK 1=失联 2=低压 3=超期 4=堵转
 *   H             重打帮助
 *
 * 验收对照（规划 §3.3/§3.6）：
 *   - A<id> 后看缓动：SG90 通道（ID3/4/5，示波器看 TIM5 引脚 PA0/1/2）
 *     脉冲从步进变恒定；到位后 Act_IsSettled=true；
 *   - 手抵大臂（ID1）：电流 > 阈值持续 300ms → stalled=true + 自动卸力
 *     + ACT_STALL 音型（L4 同款声光）；轻拍/瞬时冲击不触发；
 *   - 爪（ID4）持续供电 20s → NeedsCooldown=true 且不强制卸力（S 查看）；
 *     R4 卸力 5s 后自动解除；
 *   - alarm 联验（不占独立编号，随本台）：L1 → 双短哔 + LED1 闪码；
 *     回 L0 静音。真实失联注入 = 停发射频，等 W2.5 daemon 接线后验全链。
 */
#include "test_bench.h"
#include "servo.h"
#include "power.h"
#include "actuator.h"
#include "alarm.h"
#include "bsp_sys.h"
#include "bsp_log.h"
#include "bsp_vofa.h"
#include "robot_config.h"
#include <stdlib.h>

/* 五实例由本测试台持有（W2.5 起移交 grab_task） */
static Actuator_t s_act[SERVO_COUNT];

/* ID1/ID2 电流判据注入源（k=0 大臂 / k=1 小臂，与 bsp 采样路序一致） */
static float CurJ0(void) { return Power_GetJointCurrent(0u); }
static float CurJ1(void) { return Power_GetJointCurrent(1u); }

static void Help(void);

/* 0.1 定点打印角度（newlib-nano 无 %f；-12.3 → "-12.3"） */
static void Log_DegX10(float deg)
{
    long x10 = (long)(deg * 10.0f);
    bool neg = (x10 < 0);
    unsigned long a = (unsigned long)(neg ? -x10 : x10);
    Log_Printf("%c%lu.%lu", neg ? '-' : '+', a / 10uL, a % 10uL);
}

static const char *StateName(ActState_t s)
{
    switch (s)
    {
    case ACT_IDLE:     return "IDLE";
    case ACT_MOVING:   return "MOVING";
    case ACT_HOLDING:  return "HOLD ";
    case ACT_RELEASED: return "REL  ";
    default:           return "?";
    }
}

/* 实例默认配置：业务软限位暂用 servo 层同款占位（标定后随宏收紧） */
static const float s_lim_min[SERVO_COUNT] = ROBOT_SERVO_LIMIT_MIN_DEG_LIST;
static const float s_lim_max[SERVO_COUNT] = ROBOT_SERVO_LIMIT_MAX_DEG_LIST;

static void FillConfig(ActConfig_t *c, ServoId_t id)
{
    c->servo_id = id;
    c->min_deg = s_lim_min[id];
    c->max_deg = s_lim_max[id];
    c->ease_dps = ROBOT_ACT_EASE_DPS;
    c->get_current_a = NULL;
    c->stall_current_a = ROBOT_ACT_STALL_CURRENT_A;
    c->stall_confirm_ms = ROBOT_ACT_STALL_CONFIRM_MS;
    c->energize_window_ms = ROBOT_ACT_ENERGIZE_WINDOW_MS;
    c->energize_max_ms = 0u;            /* 默认热保护禁用，ID4 再打开 */
    c->cooldown_ms = ROBOT_ACT_COOLDOWN_MS;
}

static void PrintStatus(void)
{
    for (uint8_t i = 0u; i < SERVO_COUNT; i++)
    {
        Log_Printf("[ACT] ID%u %s cur=", (unsigned)(i + 1u), StateName(s_act[i].state));
        Log_DegX10(s_act[i].cur_deg);
        Log_Printf(" target=");
        Log_DegX10(s_act[i].target_deg);
        Log_Printf("%s%s\r\n",
                   Act_IsStalled(&s_act[i]) ? " [STALL]" : "",
                   Act_NeedsCooldown(&s_act[i]) ? " [HOT]" : "");
    }
    float i0 = Power_GetJointCurrent(0u);
    float i1 = Power_GetJointCurrent(1u);
    Log_Printf("[ACT] J0=%ldmA J1=%ldmA alarm=%d\r\n",
               (long)(i0 * 1000.0f), (long)(i1 * 1000.0f), (int)Alarm_Get());
}

/* "[-]digits" 保守解析（VOFA+ 控件冒号分隔一并兼容） */
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
    case 'A': case 'a':
    {
        const char *p = SkipNonDigit(line + 1);
        long id = strtol(p, (char **)&p, 10);
        float deg = strtof(SkipNonDigit(p), NULL);
        if (id < 1 || id > SERVO_COUNT)
        {
            Log_Printf("[ACT] A 参数非法（id=1..%d），如 A1 90\r\n", (int)SERVO_COUNT);
            break;
        }
        Act_SetTarget(&s_act[id - 1u], deg);
        Log_Printf("[ACT] ID%ld → ", id);
        Log_DegX10(deg);
        Log_Printf("°（缓动 %ld°/s，软限位 %ld..%ld）\r\n",
                   (long)ROBOT_ACT_EASE_DPS,
                   (long)s_lim_min[id - 1u], (long)s_lim_max[id - 1u]);
        break;
    }
    case 'R': case 'r':
    {
        long id = strtol(SkipNonDigit(line + 1), NULL, 10);
        if (id < 1 || id > SERVO_COUNT)
        {
            Log_Printf("[ACT] R 参数非法（id=1..%d）\r\n", (int)SERVO_COUNT);
            break;
        }
        Act_Release(&s_act[id - 1u]);
        Log_Printf("[ACT] ID%ld 已卸力\r\n", id);
        break;
    }
    case 'S': case 's':
        PrintStatus();
        break;
    case 'L': case 'l':
    {
        long n = strtol(SkipNonDigit(line + 1), NULL, 10);
        if (n < 0 || n > (long)ALARM_ACT_STALL)
        {
            Log_Printf("[ACT] L 参数非法（0..%d）\r\n", (int)ALARM_ACT_STALL);
            break;
        }
        Alarm_Set((AlarmCode_t)n);
        Log_Printf("[ACT] 告警码 → %ld\r\n", n);
        break;
    }
    case 'H': case 'h':
        Help();
        break;
    default:
        Log_Printf("[ACT] 未知命令 '%s'\r\n", line);
        Help();
        break;
    }
}

static void Help(void)
{
    Log_Printf("[ACT] 10=ACTUATOR（命令走 UART4 921600 文本行）：\r\n");
    Log_Printf("[ACT]   A<id> <deg> 设目标(缓动)  R<id> 卸力  S 查状态\r\n");
    Log_Printf("[ACT]   L<n> 注入告警码(0=OK 1=失联 2=低压 3=超期 4=堵转)  H 帮助\r\n");
    Log_Printf("[ACT] 策略：ID1/2 电流堵转判据 | ID4 热保护(20s/30s 窗) | ID3/5 无判据\r\n");
}

void Test_Actuator_Init(void)
{
    Servo_InitAll();
    Power_Init();       /* 关节电流判据源（幂等，J0/J1 双路启动） */
    Bsp_Vofa_Init();    /* UART4 @921600 + 命令接收入口 */

    ActConfig_t c;
    FillConfig(&c, SERVO_ID1);
    c.get_current_a = CurJ0;
    (void)Act_Init(&s_act[SERVO_ID1], &c);

    FillConfig(&c, SERVO_ID2);
    c.get_current_a = CurJ1;
    (void)Act_Init(&s_act[SERVO_ID2], &c);

    FillConfig(&c, SERVO_ID3);                          /* 手腕：全无 */
    (void)Act_Init(&s_act[SERVO_ID3], &c);

    FillConfig(&c, SERVO_ID4);
    c.energize_max_ms = ROBOT_ACT_ENERGIZE_MAX_MS;      /* 爪开合：热保护使能 */
    (void)Act_Init(&s_act[SERVO_ID4], &c);

    FillConfig(&c, SERVO_ID5);                          /* 爪旋转：全无 */
    (void)Act_Init(&s_act[SERVO_ID5], &c);

    Alarm_Set(ALARM_OK);
    Log_Printf("[ACT] 10=ACTUATOR 就绪（中位稳定窗 300ms 过后可控，缓动 %ld°/s）\r\n",
               (long)ROBOT_ACT_EASE_DPS);
    Help();
}

void Test_Actuator_Poll(void)
{
    /* 命令优先（操作不等节拍） */
    char line[ROBOT_VOFA_RX_LINE];
    while (Bsp_Vofa_ReadLine(line, (uint8_t)sizeof line))
    {
        HandleCommand(line);
    }

    /* 五实例统一 10ms 节拍驱动（判据/累计器全在 Act_Update 内闭环） */
    uint32_t now = Bsp_GetMs();
    for (uint8_t i = 0u; i < SERVO_COUNT; i++)
    {
        Act_Update(&s_act[i], now);
    }

    /* alarm 节奏发生器（按绝对时间推进，10ms/100ms 调用皆正确） */
    Alarm_Poll();
}
