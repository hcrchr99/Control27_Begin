/**
 * @file    remote_acceptance.c
 * @brief   【临时】S2 车端验收任务：强覆盖 freertos.c 的 __weak StartApp_Remote_Task
 *
 * 覆盖验收项（规划第 6 节第一段：车端单板自验）：
 *  A1 STATUS 上电复位值 0x0E          → SPI 总线 + CSN 时序正确
 *  A2 RF_CH 回写回读一致              → 寄存器读写通
 *  A3 Remote_Init(PRX) + 寄存器 dump  → 模式配置正确
 *  A4 无发送源时 Remote_IsLinkUp 恒 false → 看门狗逻辑正确
 * 第二段（双板联调）：递增计数 / 收包率 ≥95% / 看门狗翻转。
 *
 * ⚠ 验收通过后整文件删除（工作流：临时验收代码验收通过后清理）。
 * 分层合法性：UserApp → Modules 头文件（remote.h/nrf24.h）向下引用合规。
 */
#include "cmsis_os.h"
#include "bsp_log.h"
#include "bsp_sys.h"
#include "remote.h"
#include "nrf24.h"
#include "robot_config.h"

/* A1/A2：直接走 nrf24 寄存器层，与 Remote_Init 的机内校验互为独立证据 */
static bool PreCheck(void)
{
    uint8_t st = Nrf24_GetStatus();
    Log_Printf("[A1] STATUS reset=0x%02X (expect 0x0E)\r\n", st);
    if (st != 0x0Eu)
    {
        return false;
    }

    Nrf24_WriteReg(NRF_REG_RF_CH, 0x5Au);
    uint8_t back = Nrf24_ReadReg(NRF_REG_RF_CH);
    Log_Printf("[A2] RF_CH wr=0x5A rd=0x%02X\r\n", back);
    return (back == 0x5Au);     /* 复原由 Remote_Init 重写全组寄存器完成 */
}

/* A3：PRX 配置态全组寄存器 dump，与 robot_config.h 占位值逐项目视核对 */
static void DumpPrxRegs(void)
{
    Log_Printf("[A3] CONFIG=0x%02X EN_AA=0x%02X EN_RX=0x%02X AW=%u RETR=0x%02X "
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

void StartApp_Remote_Task(void const * argument)
{
    (void)argument;
    osDelay(100u);      /* 等 Bsp/Log 初始化与模块电源稳定 */

    if (!PreCheck())
    {
        Log_Printf("[S2] PreCheck FAIL: SPI bus / wiring / common-GND\r\n");
        for (;;) { osDelay(1000u); }
    }

    if (!Remote_Init(REMOTE_MODE_PRX))
    {
        Log_Printf("[A3] Remote_Init FAIL\r\n");
        for (;;) { osDelay(1000u); }
    }
    DumpPrxRegs();
    Log_Printf("[A3] Remote_Init(PRX) OK\r\n");

    /* 主循环：帧到达即打印首字节序号；每秒统计一次收包率与链路状态 */
    uint8_t  frame[ROBOT_REMOTE_PAYLOAD];
    uint32_t last_stat_ms = Bsp_GetMs();
    uint32_t last_count   = 0u;

    for (;;)
    {
        Remote_Service();

        while (Remote_ReadPacket(frame, (uint8_t)sizeof frame))
        {
            /* 测试发送包约定：frame[0] = PTX 递增计数（回绕），丢包看序号跳变 */
            Log_Printf("RX seq=%u total=%u\r\n", frame[0], Remote_GetRxCount());
        }

        uint32_t now = Bsp_GetMs();
        if ((now - last_stat_ms) >= 1000u)
        {
            uint32_t cnt = Remote_GetRxCount();
            Log_Printf("[STAT] %upkts/s total=%u link=%d\r\n",
                       (unsigned)(cnt - last_count), (unsigned)cnt,
                       (int)Remote_IsLinkUp());
            last_count = cnt;
            last_stat_ms = now;
        }
    }
}
