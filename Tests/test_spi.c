/**
 * @file    test_spi.c
 * @brief   S2 测试项：SPI 通路检查（nRF24 STATUS 复位值 + RF_CH 回写回读）
 *
 * 验收标准（commit 136956e 第一段 A1/A2）：
 *  A1 STATUS 上电复位值 = 0x0E   → SPI 总线 + CSN 时序正确
 *  A2 RF_CH 写 0x5A 回读一致     → 寄存器读写通
 * 一次性测试，结果打完即转入空转；与 remote_acceptance 互斥由
 * TestBench_Yield 保证（测试台激活时 remote 任务让位，SPI 独占）。
 * ⚠ 失败排查顺序：共地 → 接线 → CSN/CE 电平 → 供电（nRF24 必须独立 3.3V 稳）。
 */
#include "test_bench.h"
#include "bsp_spi.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "nrf24.h"

void Test_Spi_Init(void)
{
    if (!Spi_Init())
    {
        Log_Printf("[T-SPI] FAIL: Spi_Init 自检不过（hspi2 未就绪）\r\n");
        return;
    }

    uint8_t st = Nrf24_GetStatus();
    Log_Printf("[T-SPI] [A1] STATUS reset=0x%02X (expect 0x0E)\r\n", st);
    if (st == 0x00u || st == 0xFFu)
    {
        Log_Printf("[T-SPI] FAIL: 全 0x00/0xFF = MISO 悬空/未共地/CSN 时序错\r\n");
        return;
    }

    Nrf24_WriteReg(NRF_REG_RF_CH, 0x5Au);
    uint8_t back = Nrf24_ReadReg(NRF_REG_RF_CH);
    Log_Printf("[T-SPI] [A2] RF_CH wr=0x5A rd=0x%02X\r\n", back);
    Nrf24_WriteReg(NRF_REG_RF_CH, (uint8_t)ROBOT_RF_CHANNEL);   /* 复原 */

    Log_Printf("[T-SPI] %s\r\n", (back == 0x5Au) ? "PASS" : "FAIL: 回读不一致");
}

void Test_Spi_Poll(void)
{
    /* 一次性测试，无轮询动作；心跳在 test_bench.c 统一处理 */
}
