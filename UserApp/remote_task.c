/**
 * @file    remote_task.c
 * @brief   车端遥控接收服务任务：PRX 服务循环 + RC_Cmd 唯一写者
 *
 * 职责：驱动 Modules/remote 的 PRX 服务循环，把收到的帧交给 rc_cmd
 * （Modules/remote/rc_cmd.c，W2.2）解析为定量指令快照；看门狗超时期间
 * 以 NULL 帧驱动 Update（架构 §5.1 的"第二写者"），维持两级故障语义。
 * 诊断项（A1 体检/A2 回写/A3 dump/逐包打印）在 Tests/test_remote.c
 * （测试台 5），rc_cmd 全链路验收在 Tests/test_rc_cmd.c（测试台 11）。
 *
 * 并发契约（架构 §5.1）：本任务是 RC_Cmd 的唯一写者；消费者（chassis/grab）
 * 一律走 RC_Cmd_GetCopy 快照，禁止触碰本模块内部状态。
 */
#include "cmsis_os.h"
#include "bsp_log.h"
#include "bsp_sys.h"
#include "rc_cmd.h"
#include "remote.h"
#include "robot_config.h"
#include "test_bench.h"

void StartApp_Remote_Task(void const * argument);

/* 累计收帧数（1Hz 统计用；帧内容消费已移交 rc_cmd） */
static uint32_t s_total;

void StartApp_Remote_Task(void const * argument)
{
    (void)argument;
    osDelay(100u);      /* 等 Bsp/Log 初始化与模块电源稳定 */
    TestBench_Yield("remote");  /* 测试台激活时让位（外设归测试台） */

    if (!Remote_Init(REMOTE_MODE_PRX))
    {
        /* Remote_Init 内部含总线存活判据（0x00/0xFF 判死）与配置回读校验，
         * 详见 Tests/test_remote.c 的完整体检项 */
        Log_Printf("[RX-TASK] Remote_Init FAIL: SPI/接线/共地排查\r\n");
        for (;;) { osDelay(1000u); }
    }
    RC_Cmd_Init();      /* 指令态清零：valid=false，消费者在首帧前拿到安全快照 */
    Log_Printf("[RX-TASK] init OK CH%d ack=%d rc_cmd=%uB/%u关节\r\n",
               (int)ROBOT_RF_CHANNEL, (int)ROBOT_RF_AUTO_ACK,
               (unsigned)sizeof(RCPayload_t), (unsigned)ROBOT_CMD_JOINT_COUNT);

    RCPayload_t frame;
    uint32_t last_stat_ms = Bsp_GetMs();

    for (;;)
    {
        Remote_Service();

        /* 唯一写者路径：收到的帧 → rc_cmd 解析（pack(1) 结构体整帧直传） */
        while (Remote_ReadPacket((uint8_t *)&frame, (uint8_t)sizeof frame))
        {
            RC_Cmd_Update(&frame, Bsp_GetMs());
            s_total++;
        }

        uint32_t now = Bsp_GetMs();
        if (!Remote_IsLinkUp())
        {
            /* 第二写者：看门狗超时，写失效值并驱动 300ms 渐停（防甩矿） */
            RC_Cmd_Update(NULL, now);
        }

        if ((now - last_stat_ms) >= 1000u)
        {
            static uint32_t s_last_count;
            RC_Cmd_t snap;
            RC_Cmd_GetCopy(&snap);
            Log_Printf("[RX-TASK] %upkts/s total=%u link=%d valid=%d estop=%d\r\n",
                       (unsigned)(s_total - s_last_count), (unsigned)s_total,
                       (int)Remote_IsLinkUp(), (int)snap.valid, (int)snap.estop);
            s_last_count = s_total;
            last_stat_ms = now;
        }
    }
}
