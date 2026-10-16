/**
 * @file    chassis_task.c
 * @brief   底盘任务：1kHz 节拍（速度环 + 遥控指令业务链）——强覆盖 freertos.c 的
 *          __weak StartApp_Chassis_Task（同 remote_task.c 模式，CubeMX 文件零改动）
 *
 * 职责边界（架构 §数据流）：本任务只做节拍与"搬运工"——每拍从 RC_Cmd 快照取
 * 指令，过 chassis_ctrl 三步链（手感整形 → 物理量 → 麦轮逆解），把四轮目标
 * 转速喂给 Modules/motor；速度环算法全在 motor（Motor_SpeedLoopUpdate），
 * 本文件不碰 PWM/PID/编码器细节。
 *
 * 使能权（2026-10-13 拍板：首帧使能后常使能）：上电 STBY 保持低 = 不出力；
 * 收到第一帧有效指令（valid 且非急停）时 Motor_Enable 一次，此后不再关断。
 * 急停/失联的安全语义由 rc_cmd 承接——指令立即归零（速度环主动刹停）或
 * 300ms 渐停，坡上带矿不溜车；本任务不重复造第二套保护。
 *
 * 安全语义：使能前 ref 已是 0 = 四路保持静止；测试台激活时 TestBench_Yield
 * 让位（编码器"每通道一个读者"契约由让位保证，测试台 6 是唯一读者）。
 */
#include "cmsis_os.h"
#include "bsp_log.h"
#include "bsp_sys.h"
#include "motor.h"
#include "rc_cmd.h"
#include "chassis_ctrl.h"
#include "test_bench.h"

void StartApp_Chassis_Task(void const * argument);

/* 使能只做一次：STBY 拉高后常使能（拍板见文件头），置 true 后不再碰 */
static bool s_enabled;

void StartApp_Chassis_Task(void const * argument)
{
    (void)argument;
    osDelay(100u);              /* 等 Bsp/Log 初始化（Bsp_Init 在 main 已完成） */
    TestBench_Yield("chassis"); /* 测试台激活时让位（外设归测试台） */

    Bsp_DwtReArm();             /* J-Link 断开会清 DWT 使能（坑#15），任务级补一枪 */

    Motor_Init();               /* 幂等：PWM + STBY 保持低 + 速度环装参清态 */

    Log_Printf("[CHASSIS] 业务链接线就绪：首帧有效指令即 Enable（STBY 低不出力）\r\n");

    for (;;)
    {
        RC_Cmd_t snap;
        RC_Cmd_GetCopy(&snap);  /* 唯一读法：关调度快照，撕裂免疫 */

        /* 首帧使能（拍板：首帧使能后常使能）。急停闩住期间不使能——
         * 急停解除（松键帧到达）后自然放行，无需额外状态 */
        if (!s_enabled && snap.valid && !snap.estop)
        {
            Motor_Enable();
            s_enabled = true;
            Log_Printf("[CHASSIS] 首帧有效指令，电机使能（此后常使能）\r\n");
        }

        /* 三步链：手感整形 → 归一化转物理量 → 麦轮逆解。
         * 急停/失联的快照指令已被 rc_cmd 归零/渐停，照常走链即得安全输出 */
        Chassis_Vec_t norm = {snap.vx, snap.vy, snap.omega};
        Chassis_Vec_t shaped, phys;
        float rpm[CHASSIS_WHEEL_COUNT];
        Chassis_Shape(&norm, &shaped);
        Chassis_ToPhysical(&shaped, &phys);
        Chassis_MecanumIK(&phys, rpm);

        /* 轮序直连占位：rpm[0..3]=FL/FR/RL/RR → MOTOR_CH1..4（同
         * robot_config 通道映射表的占位纪律）；实机布线定案后只改表，
         * 电气方向修正归 ROBOT_MOTOR_SIGN / 电机接线 */
        Motor_SetSpeedRpm(MOTOR_CH1, rpm[0]);
        Motor_SetSpeedRpm(MOTOR_CH2, rpm[1]);
        Motor_SetSpeedRpm(MOTOR_CH3, rpm[2]);
        Motor_SetSpeedRpm(MOTOR_CH4, rpm[3]);

        Motor_SpeedLoopUpdate();    /* PID dt 由实例内部 Bsp_GetUs 自算 */
        osDelay(1u);                /* ≈1kHz（FreeRTOS 1kHz tick） */
    }
}
