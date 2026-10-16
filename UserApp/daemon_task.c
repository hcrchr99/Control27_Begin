/**
 * @file    daemon_task.c
 * @brief   管家任务：告警声光专属节拍（100ms）——强覆盖 freertos.c 的
 *          __weak StartApp_Daemon_Task（CubeMX 文件零改动）
 *
 * 职责边界（架构 §5.1 / alarm.h 定位）：Alarm 模块是"码 → 声光"的执行端，
 * 音型相位按绝对时间推进、与调用频率无关——Poll 的节奏归本任务（100ms 一拍），
 * 谁在什么时候 Set 什么码是各业务的事（本轮回：grab_task 堵转立 ACT_STALL/
 * 降 OK；测试台激活时由测试台自驱 Poll 并接管声光，本任务让位不抢拍）。
 *
 * 本轮为最小版：只当声光节奏发生器的管家。daemon 二期再上（架构既定）：
 * 各模块心跳收集、失联码 LINK_LOST、喂看门狗、NeedsCooldown 后何时敢
 * Act_Release 卸力掉矿的拍板时机。
 */
#include "cmsis_os.h"
#include "bsp_log.h"
#include "alarm.h"
#include "test_bench.h"

void StartApp_Daemon_Task(void const * argument);

void StartApp_Daemon_Task(void const * argument)
{
    (void)argument;
    osDelay(100u);              /* 等 Bsp/Log 初始化（Bsp_Init 在 main 已完成） */
    TestBench_Yield("daemon");  /* 测试台激活时让位（声光归测试台联验） */

    Alarm_Set(ALARM_OK);        /* 开机静音：显式落 OK 码，音型从静音起 */
    Log_Printf("[DAEMON] 告警节拍就绪（100ms Poll，码源=业务任务）\r\n");

    for (;;)
    {
        Alarm_Poll();               /* 非阻塞单次迭代：按绝对时间推音型相位 */
        osDelay(100u);
    }
}
