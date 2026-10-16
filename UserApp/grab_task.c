/**
 * @file    grab_task.c
 * @brief   机械臂任务：10ms 节拍（关节速率积分 → 五路舵机缓动执行）——
 *          强覆盖 freertos.c 的 __weak StartApp_Grab_Task（CubeMX 文件零改动）
 *
 * 职责边界（架构 §数据流）：本任务持有五路执行器实例（架构 §5.2"5 实例静态
 * 于 grab_task"）与 Grab_State_t；每拍从 RC_Cmd 快照摘出关节速率喂 grab_ctrl
 * 积分链，再把目标角交给 actuator 保护层（缓动/堵转/热保护全在模块内闭环）。
 * 本文件不碰 PWM/脉宽/电流采样细节。
 *
 * 保护矩阵（2026-10-13 随测试台 10 移交本任务，策略同 W2.4 板验定案）：
 *   ID1 大臂 / ID2 小臂：电流堵转判据（注入 Power_GetJointCurrent），热保护禁用；
 *   ID3 手腕 / ID5 爪旋转：无判据 + 热保护禁用（全程业务性保持，防线 = 轻载 + 报警 + 操作手）；
 *   ID4 爪开合：无判据 + 热保护使能（触发只报告不强制卸力——夹矿时卸不卸由操作手拍板）。
 *
 * 喂法纪律（关键）：Act_SetTarget 每次调用都会重算缓动斜坡并清堵转锁存重新
 * 起判——每拍盲喂会让 300ms 堵转确认窗永远凑不满。所以只有目标角真的变了
 * （>0.05°）才喂；速率归零后不再喂，执行器进入 HOLDING 恒发保持，堵转判据
 * 才能真正盯着供电电流干活。
 *
 * 堵转声光源：本任务是 ACT_STALL 告警码的唯一来源——任一实例堵转锁存即
 * Alarm_Set(ACT_STALL)；全部回落且当前码确为 ACT_STALL 时降回 OK（单码模型，
 * 只降自己立的码，将来 daemon 接管心跳收集后码源归边）。
 */
#include "cmsis_os.h"
#include "bsp_log.h"
#include "bsp_sys.h"
#include "rc_cmd.h"
#include "grab_ctrl.h"
#include "servo.h"
#include "power.h"
#include "actuator.h"
#include "alarm.h"
#include "test_bench.h"
#include "robot_config.h"
#include <math.h>

void StartApp_Grab_Task(void const * argument);

/* 协议关节序 = 舵机 ID 序（rc_cmd v2 冻结）：两套计数脱钩时本文件循环会
 * 静默越界，编译期钉死 */
_Static_assert(SERVO_COUNT == ROBOT_CMD_JOINT_COUNT,
               "servo count must match protocol joint count");

/* 五路执行器实例 + 积分状态 + 误差源：全部静态于本任务（架构既定），测试台 10
 * 激活时本任务整体让位，不与之并存 */
static Actuator_t s_act[SERVO_COUNT];
static Grab_State_t s_grab;

/* 目标角喂点：上一次真正喂给 Act_SetTarget 的角度（变化超阈值才再喂） */
static float s_last_set[SERVO_COUNT];

/* ID1/ID2 电流判据注入源（k=0 大臂 / k=1 小臂，与 bsp 采样路序一致） */
static float CurJ0(void) { return Power_GetJointCurrent(0u); }
static float CurJ1(void) { return Power_GetJointCurrent(1u); }

/* 实例配置：保护参数取 robot_config 全局宏，业务软限位暂用 servo 层同款
 * 占位（实机机械限位定案后随 Servo 表一并收紧） */
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

void StartApp_Grab_Task(void const * argument)
{
    (void)argument;
    osDelay(100u);              /* 等 Bsp/Log 初始化（Bsp_Init 在 main 已完成） */
    TestBench_Yield("grab");    /* 测试台激活时让位（舵机/电流采样归测试台） */

    Bsp_DwtReArm();             /* J-Link 断开会清 DWT 使能（坑#15），任务级补一枪 */

    Servo_InitAll();            /* 上电安全顺序：先发中位（坑#5），后使能业务 */
    Power_Init();               /* ID1/ID2 电流判据源（幂等，J0/J1 双路启动） */

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

    /* 积分链起点 = 舵机中位（Act_Init 记 90° 为当前位置，两表一致才不跳） */
    const float deg0[ROBOT_CMD_JOINT_COUNT] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
    Grab_Init(&s_grab, deg0);
    for (uint8_t i = 0u; i < SERVO_COUNT; i++)
    {
        s_last_set[i] = s_grab.deg[i];
    }

    Log_Printf("[GRAB] 五关节就绪（10ms 拍，缓动 %ld°/s，积分 %ld°/s）\r\n",
               (long)ROBOT_ACT_EASE_DPS, (long)ROBOT_APP_JOINT_RATE_DPS);

    for (;;)
    {
        RC_Cmd_t snap;
        RC_Cmd_GetCopy(&snap);  /* 唯一读法：关调度快照，撕裂免疫 */

        uint32_t now = Bsp_GetMs();
        Grab_Update(&s_grab, snap.joint, now);  /* 急停/失联语义已内含在速率里 */

        /* 变化才喂（纪律见文件头）：SetTarget 盲喂会重置堵转确认窗 */
        for (uint8_t i = 0u; i < SERVO_COUNT; i++)
        {
            if (fabsf(s_grab.deg[i] - s_last_set[i]) > 0.05f)
            {
                Act_SetTarget(&s_act[i], s_grab.deg[i]);
                s_last_set[i] = s_grab.deg[i];
            }
        }

        /* 五实例统一 10ms 节拍驱动（缓动/堵转/热累计全在 Act_Update 内闭环） */
        for (uint8_t i = 0u; i < SERVO_COUNT; i++)
        {
            Act_Update(&s_act[i], now);
        }

        /* 堵转声光（本任务 = ACT_STALL 唯一码源，降码只降自己立的） */
        bool any_stall = false;
        for (uint8_t i = 0u; i < SERVO_COUNT; i++)
        {
            if (Act_IsStalled(&s_act[i]))
            {
                any_stall = true;
                break;
            }
        }
        if (any_stall)
        {
            Alarm_Set(ALARM_ACT_STALL);
        }
        else if (Alarm_Get() == ALARM_ACT_STALL)
        {
            Alarm_Set(ALARM_OK);
        }

        osDelay(10u);               /* actuator 规划节拍 10~20ms，取下限跟手 */
    }
}
