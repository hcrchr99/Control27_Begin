/**
 * @file    ptx_test.c
 * @brief   S2 双板联调测试发送端（F103C8 最小系统板）：100Hz 发 15B 递增计数包
 *
 * 入口经 freertos.c 的 StartDefaultTask USER CODE 区转发到 Ptx_Test_Main
 * （defaultTask 是 CubeMX 生成的强符号，不能像车端任务那样用弱符号覆盖）。
 * 代码路径与未来遥控器一致：完整 Bsp 移植版 + Modules/remote（PTX 模式），
 * IRQ 经 Exti_Attach 注册、osSignalSet 唤醒 SendPacket——车端收包率统计的
 * 发送基准即本固件的发包序号 frame[0]（uint8 回绕，掉包看序号跳变）。
 *
 * 车端对侧验收（UserApp/remote_acceptance.c 主循环）：
 *  - 递增计数连续无跳变；收包率 = 车端 GetRxCount / 本端 attempts ≥95%；
 *  - 停发（按住本板复位）→ 车端 link 400ms 内翻 false，恢复后自动回 true。
 */
#include "cmsis_os.h"
#include "bsp_sys.h"
#include "bsp_log.h"
#include "remote.h"
#include "nrf24.h"
#include "robot_config.h"

void Ptx_Test_Main(void const * argument);

/* RF 物理层体检：连续载波 5 秒（附录 C：CONT_WAVE + PLL_LOCK + CE 常高）。
 * 对端 PRX 在这 5 秒内看 rpd：rpd=1 ⇒ 天线/距离/频段物理层全通；
 * rpd=0 ⇒ 硬件问题（天线/供电/模块/频率），与收发包逻辑无关。 */
static void CarrierTest(void)
{
    uint8_t rf = Nrf24_ReadReg(NRF_REG_RF_SETUP);
    Nrf24_WriteReg(NRF_REG_RF_SETUP,
                   (uint8_t)(rf | NRF_RF_SETUP_CONT_WAVE | NRF_RF_SETUP_PLL_LOCK));
    Spi_Ce(true);
    for (uint32_t i = 5u; i > 0u; i--)
    {
        Log_Printf("[PTX] 载波测试 %us...（看对端 rpd）\r\n", (unsigned)i);
        osDelay(1000u);
    }
    Spi_Ce(false);
    Nrf24_WriteReg(NRF_REG_RF_SETUP, rf);   /* 复原，正常发包模式 */
}

void Ptx_Test_Main(void const * argument)
{
    (void)argument;
    Bsp_Init();
    Log_Init();
    osDelay(100u);      /* 等模块电源稳定 */

    if (!Remote_Init(ROBOT_DIAG_ROLE_SWAP ? REMOTE_MODE_PRX : REMOTE_MODE_PTX))
    {
        Log_Printf("[PTX] Remote_Init FAIL: SPI/接线/共地排查\r\n");
        for (;;) { osDelay(1000u); }
    }
    Log_Printf("[PTX] init OK: %s CH%d\r\n",
               ROBOT_DIAG_ROLE_SWAP ? "诊断互换:本板收(PRX)" : "15B @ 100Hz 发送",
               (int)ROBOT_RF_CHANNEL);

#if ROBOT_DIAG_ROLE_SWAP
    /* 频偏定位扫描：对端持续发射中，逐频道采样 RPD 找能量真实落点。
     * 若对端实际中心偏离标称频道（晶振偏差/兼容片频偏），rpd=1 的频道
     * 就是它的真实落点；锁到命中频道后进入接收。 */
    {
        Nrf24_FlushRx();
        uint8_t hits[13] = {0};
        for (int round = 0; round < 5; round++)
        {
            for (uint8_t ch = 70u; ch <= 82u; ch++)
            {
                Nrf24_WriteReg(NRF_REG_RF_CH, ch);
                osDelay(2u);    /* PLL 重锁（130us）+ RPD 稳定（40us）+ 余量 */
                hits[ch - 70u] += (uint8_t)(Nrf24_ReadReg(NRF_REG_RPD) & 0x01u);
            }
        }
        uint8_t best = ROBOT_RF_CHANNEL;
        int best_hits = -1;
        for (int i = 0; i < 13; i++)
        {
            Log_Printf("[C8-RX] sweep ch=%d rpd_hits=%d/5\r\n", 70 + i, hits[i]);
            if (hits[i] > best_hits) { best_hits = hits[i]; best = (uint8_t)(70 + i); }
        }
        Nrf24_WriteReg(NRF_REG_RF_CH, best);
        Nrf24_FlushRx();
        Log_Printf("[C8-RX] LOCK ch=%d (标称 %d, 偏差 %+d MHz)%s\r\n",
                   best, (int)ROBOT_RF_CHANNEL, (int)best - (int)ROBOT_RF_CHANNEL,
                   (best == ROBOT_RF_CHANNEL) ? "" : " ← 频偏实锤");
    }

    /* 诊断互换：测试板临时当接收端，验证测试板模块的接收能力 */
    {
        uint8_t  frame[ROBOT_REMOTE_PAYLOAD];
        uint32_t total = 0u, last_stat_ms = Bsp_GetMs(), last_count = 0u;

        for (;;)
        {
            Remote_Service();
            while (Remote_ReadPacket(frame, (uint8_t)sizeof frame))
            {
                Log_Printf("RX seq=%u total=%u\r\n", frame[0],
                           (unsigned)Remote_GetRxCount());
            }
            uint32_t now = Bsp_GetMs();
            if ((now - last_stat_ms) >= 1000u)
            {
                uint32_t cnt = Remote_GetRxCount();
                Log_Printf("[C8-RX] %upkts/s total=%u rpd=%d st=0x%02X\r\n",
                           (unsigned)(cnt - last_count), (unsigned)cnt,
                           (int)(Nrf24_ReadReg(NRF_REG_RPD) & 0x01u),
                           Nrf24_GetStatus());
                last_count = cnt;
                last_stat_ms = now;
            }
        }
    }
#endif

    /* TX FIFO 写入校验：写一包后 TX_EMPTY(bit6) 必须变 0，
     * 仍为 1 = 载荷根本没进 FIFO（SPI 写路径/模块问题） */
    {
        uint8_t probe[ROBOT_REMOTE_PAYLOAD] = {0};
        Nrf24_WriteRegBuf(NRF_CMD_W_TX_PAYLOAD, probe, (uint8_t)sizeof probe);
        uint8_t fifo = Nrf24_ReadReg(NRF_REG_FIFO_STATUS);
        Log_Printf("[PTX] fifo_chk=0x%02X (TX_EMPTY=%d, 期望 0)\r\n",
                   fifo, (int)((fifo & NRF_FIFO_TX_EMPTY) != 0u));
        Nrf24_FlushTx();
    }

    CarrierTest();

    uint8_t  frame[ROBOT_REMOTE_PAYLOAD] = {0};
    uint32_t seq = 0u;
    uint32_t attempts = 0u;
    uint32_t ack_ok = 0u;
    uint32_t last_stat_ms = Bsp_GetMs();

    for (;;)
    {
        frame[0] = (uint8_t)(++seq);    /* 发送序号（车端打印 frame[0] 对账） */
        attempts++;
        if (Remote_SendPacket(frame, (uint8_t)sizeof frame))
        {
            ack_ok++;
        }

        uint32_t now = Bsp_GetMs();
        if ((now - last_stat_ms) >= 1000u)
        {
            /* st=STATUS arc=ARC_CNT(本包重传次数) los=PLOS_CNT(累计丢包)：
             * arc/los 恒 0 = 从未发射（查 CE 接线/PWR_UP）；
             * los 增长 = 发射了但收不到 ACK（查天线/供电/共地/距离/频道） */
            uint8_t st = Nrf24_GetStatus();
            uint8_t obs = Nrf24_ReadReg(NRF_REG_OBSERVE_TX);
            Log_Printf("[PTX] att=%u ok=%u fail=%u seq=%u st=0x%02X arc=%u los=%u\r\n",
                       (unsigned)attempts, (unsigned)ack_ok,
                       (unsigned)Remote_GetTxFailCount(), (unsigned)seq,
                       st, (unsigned)(obs & 0x0Fu), (unsigned)(obs >> 4));
            last_stat_ms = now;
        }
        osDelay(10u);       /* ≈100Hz；实际节奏 = 10ms + 发送阻塞时长，两端以计数对账 */
    }
}
