/**
 * @file    chassis_task.c
 * @brief   底盘任务：1kHz 速度环节拍（W2.3 骨架）——强覆盖 freertos.c 的
 *          __weak StartApp_Chassis_Task（同 remote_task.c 模式，CubeMX 文件零改动）
 *
 * 职责边界（架构 §数据流）：本任务只提供控制节拍与（后续 W2.4+ 的）
 * RC_Cmd_GetCopy → 底盘逆解 → Motor_SetSpeedRpm 业务链；速度环算法全在
 * Modules/motor（Motor_SpeedLoopUpdate），本文件不碰 PWM/PID/编码器细节。
 *
 * 安全语义：ref 默认 0 = 四路保持静止；不调 Motor_Enable（STBY 低不出力），
 * 使能权留给上层/测试台；测试台激活时 TestBench_Yield 让位（编码器"每通道
 * 一个读者"契约由让位保证，测试台 6 是唯一读者）。
 */
#include "cmsis_os.h"
#include "bsp_log.h"
#include "bsp_sys.h"
#include "motor.h"
#include "test_bench.h"

void StartApp_Chassis_Task(void const * argument);

void StartApp_Chassis_Task(void const * argument)
{
    (void)argument;
    osDelay(100u);              /* 等 Bsp/Log 初始化（Bsp_Init 在 main 已完成） */
    TestBench_Yield("chassis"); /* 测试台激活时让位（外设归测试台） */

    Bsp_DwtReArm();             /* J-Link 断开会清 DWT 使能（坑#15），任务级补一枪 */

    Motor_Init();               /* 幂等：PWM + STBY 保持低 + 速度环装参清态 */

    Log_Printf("[CHASSIS] 速度环 1kHz 节拍就绪 ref=0（STBY 低不出力）\r\n");

    for (;;)
    {
        Motor_SpeedLoopUpdate();    /* PID dt 由实例内部 Bsp_GetUs 自算 */
        osDelay(1u);                /* ≈1kHz（FreeRTOS 1kHz tick） */
    }
}
