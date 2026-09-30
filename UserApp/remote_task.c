/**
 * @file    remote_task.c
 * @brief   车端遥控接收服务任务（remote_acceptance.c 验收完毕后的正式形态）
 *
 * 职责：驱动 Modules/remote 的 PRX 服务循环，消费遥控帧并维护链路统计。
 * 诊断项（A1 总线体检/A2 回写回读/A3 寄存器 dump/逐包打印）已迁至
 * Tests/test_remote.c（测试台 TEST_BENCH_REMOTE 项），本文件只保留业务路径。
 *
 * 二期接缝：控制逻辑（速度/按键解析）在主循环的 ReadPacket 处接入；
 * s_latest/s_total 即最新帧快照与累计计数（同一任务读写，无锁）。
 */
#include "cmsis_os.h"
#include <string.h>
#include "bsp_log.h"
#include "bsp_sys.h"
#include "remote.h"
#include "robot_config.h"
#include "test_bench.h"

void StartApp_Remote_Task(void const * argument);

/* 最新帧快照（UserApp 二期控制逻辑的读取点） */
static uint8_t  s_latest[ROBOT_REMOTE_PAYLOAD];
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
    Log_Printf("[RX-TASK] init OK CH%d ack=%d\r\n",
               (int)ROBOT_RF_CHANNEL, (int)ROBOT_RF_AUTO_ACK);

    uint8_t  frame[ROBOT_REMOTE_PAYLOAD];
    uint32_t last_stat_ms = Bsp_GetMs();

    for (;;)
    {
        Remote_Service();

        while (Remote_ReadPacket(frame, (uint8_t)sizeof frame))
        {
            /* 二期接缝：此处把 frame 交给控制逻辑（速度/按键解析） */
            memcpy(s_latest, frame, sizeof s_latest);
            s_total++;
        }

        uint32_t now = Bsp_GetMs();
        if ((now - last_stat_ms) >= 1000u)
        {
            static uint32_t s_last_count;
            Log_Printf("[RX-TASK] %upkts/s total=%u link=%d\r\n",
                       (unsigned)(s_total - s_last_count), (unsigned)s_total,
                       (int)Remote_IsLinkUp());
            s_last_count = s_total;
            last_stat_ms = now;
        }
    }
}
