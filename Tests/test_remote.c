/**
 * @file    test_remote.c
 * @brief   S2 测试项：无线链路收发体检（remote_acceptance.c 的验收语义迁入）
 *
 * 覆盖验收项（commit 136956e / bf64d02）：
 *  A1 总线存活判据（0x00/0xFF 判死；0x40=模块未断电残留 RX_DR，重配置可恢复）
 *  A2 RF_CH 回写回读
 *  A3 Remote_Init(PRX) + 寄存器全组 dump
 *  收包循环 + 每秒统计（丢包观察用）
 */
#include "test_bench.h"
#include "cmsis_os.h"
#include "bsp_log.h"
#include "bsp_sys.h"
#include "remote.h"
#include "nrf24.h"
#include "robot_config.h"

static bool s_ok;

static void DumpPrxRegs(void)
{
    Log_Printf("[T-REM] [A3] CONFIG=0x%02X EN_AA=0x%02X EN_RX=0x%02X AW=%u RETR=0x%02X "
               "RF_CH=%u RF_SETUP=0x%02X RX_PW=%u FIFO=0x%02X STATUS=0x%02X\r\n",
               Nrf24_ReadReg(NRF_REG_CONFIG),
               Nrf24_ReadReg(NRF_REG_EN_AA),
               Nrf24_ReadReg(NRF_REG_EN_RXADDR),
               (unsigned)(Nrf24_ReadReg(NRF_REG_SETUP_AW) + 2u),
               Nrf24_ReadReg(NRF_REG_SETUP_RETR),
               Nrf24_ReadReg(NRF_REG_RF_CH),
               Nrf24_ReadReg(NRF_REG_RF_SETUP),
               Nrf24_ReadReg(NRF_REG_RX_PW_P0),
               Nrf24_ReadReg(NRF_REG_FIFO_STATUS),
               Nrf24_ReadReg(NRF_REG_STATUS));
}

void Test_Remote_Init(void)
{
    s_ok = false;

    /* A1：总线存活判据（0x00/0xFF = MISO 死/CSN 错/未共地；0x40 = 模块未断电
     * 残留 RX_DR，重配置可恢复，详见 commit bf64d02） */
    uint8_t st = Nrf24_GetStatus();
    Log_Printf("[T-REM] [A1] STATUS=0x%02X%s\r\n", st,
               (st == 0x0Eu) ? " (上电复位值)" :
               (st == 0x40u) ? " (模块未断电：残留 RX_DR)" : " (异常值)");
    if (st == 0x00u || st == 0xFFu)
    {
        return;
    }

    /* A2：寄存器回写回读（RF_CH 复原由 Remote_Init 重写全组完成） */
    Nrf24_WriteReg(NRF_REG_RF_CH, 0x5Au);
    uint8_t back = Nrf24_ReadReg(NRF_REG_RF_CH);
    Log_Printf("[T-REM] [A2] RF_CH wr=0x5A rd=0x%02X\r\n", back);
    if (back != 0x5Au)
    {
        return;
    }

    if (!Remote_Init(REMOTE_MODE_PRX))
    {
        Log_Printf("[T-REM] [A3] Remote_Init FAIL\r\n");
        return;
    }
    DumpPrxRegs();
    Log_Printf("[T-REM] [A3] Remote_Init(PRX) OK，收包循环开始\r\n");
    s_ok = true;
}

void Test_Remote_Poll(void)
{
    static uint8_t  frame[ROBOT_REMOTE_PAYLOAD];
    static uint32_t last_stat_ms;
    static uint32_t last_count;

    if (!s_ok)
    {
        return;
    }

    Remote_Service();

    while (Remote_ReadPacket(frame, (uint8_t)sizeof frame))
    {
        /* 首字节为发送端递增序号，跳变即丢包 */
        Log_Printf("[T-REM] RX seq=%u total=%u\r\n",
                   frame[0], (unsigned)Remote_GetRxCount());
    }

    uint32_t now = Bsp_GetMs();
    if ((now - last_stat_ms) >= 1000u)
    {
        uint32_t cnt = Remote_GetRxCount();
        Log_Printf("[T-REM] [STAT] %upkts/s total=%u link=%d rpd=%d ch=%d rf=0x%02X st=0x%02X\r\n",
                   (unsigned)(cnt - last_count), (unsigned)cnt,
                   (int)Remote_IsLinkUp(),
                   (int)(Nrf24_ReadReg(NRF_REG_RPD) & 0x01u),
                   (int)Nrf24_ReadReg(NRF_REG_RF_CH),
                   Nrf24_ReadReg(NRF_REG_RF_SETUP),
                   Nrf24_GetStatus());
        last_count = cnt;
        last_stat_ms = now;
    }
}
