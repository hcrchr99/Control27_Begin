/**
 * @file    nrf24.c
 * @brief   nRF24L01+ 寄存器级驱动实现
 *
 * 总线时序统一为：CSN 选中 → 指令字节（读时第 2 拍起为数据）→ CSN 释放。
 * 每拍经 Spi_TransferByte 全双工完成，指令与 STATUS 回读同拍。
 */
#include "nrf24.h"
#include "bsp_spi.h"
#include "bsp_sys.h"
#include <stddef.h>

/* ============================== 寄存器访问 ================================ */

uint8_t Nrf24_ReadReg(uint8_t reg)
{
    Spi_Csn(true);
    Spi_TransferByte(NRF_CMD_R_REGISTER | (reg & 0x1Fu));
    uint8_t val = Spi_TransferByte(NRF_CMD_NOP);
    Spi_Csn(false);
    return val;
}

void Nrf24_ReadRegBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Spi_Csn(true);
    Spi_TransferByte(NRF_CMD_R_REGISTER | (reg & 0x1Fu));
    for (uint8_t i = 0u; i < len; i++)
    {
        buf[i] = Spi_TransferByte(NRF_CMD_NOP);
    }
    Spi_Csn(false);
}

void Nrf24_WriteReg(uint8_t reg, uint8_t val)
{
    Spi_Csn(true);
    Spi_TransferByte(NRF_CMD_W_REGISTER | (reg & 0x1Fu));
    Spi_TransferByte(val);
    Spi_Csn(false);
}

void Nrf24_WriteRegBuf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    Spi_Csn(true);
    Spi_TransferByte(NRF_CMD_W_REGISTER | (reg & 0x1Fu));
    for (uint8_t i = 0u; i < len; i++)
    {
        (void)Spi_TransferByte(buf[i]);
    }
    Spi_Csn(false);
}

uint8_t Nrf24_GetStatus(void)
{
    Spi_Csn(true);
    uint8_t st = Spi_TransferByte(NRF_CMD_NOP);
    Spi_Csn(false);
    return st;
}

void Nrf24_FlushTx(void)
{
    Spi_Csn(true);
    (void)Spi_TransferByte(NRF_CMD_FLUSH_TX);
    Spi_Csn(false);
}

void Nrf24_FlushRx(void)
{
    Spi_Csn(true);
    (void)Spi_TransferByte(NRF_CMD_FLUSH_RX);
    Spi_Csn(false);
}

/* ============================== 配置与校验 ================================ */

bool Nrf24_Configure(const Nrf24_Config_t *cfg)
{
    if (cfg == NULL)
    {
        return false;
    }
    if (cfg->addr_width < 3u || cfg->addr_width > NRF_ADDR_WIDTH_MAX ||
        cfg->payload_width == 0u || cfg->payload_width > NRF_PAYLOAD_MAX ||
        cfg->channel > NRF_CHANNEL_MAX)
    {
        return false;
    }

    /* 先断电再配置：对重复 Init 同样成立（PWR_UP 期间改配置须回断电态） */
    Nrf24_WriteReg(NRF_REG_CONFIG, NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO);

    Nrf24_WriteReg(NRF_REG_EN_AA, cfg->auto_ack ? 0x01u : 0x00u);
    Nrf24_WriteReg(NRF_REG_EN_RXADDR, 0x01u);   /* 仅管道 0（单播对端） */
    Nrf24_WriteReg(NRF_REG_SETUP_AW, (uint8_t)(cfg->addr_width - 2u));
    Nrf24_WriteRegBuf(NRF_REG_RX_ADDR_P0, cfg->addr, cfg->addr_width);
    /* PTX 自动应答须 TX_ADDR == RX_ADDR_P0，PRX 写之无效也无害 */
    Nrf24_WriteRegBuf(NRF_REG_TX_ADDR, cfg->addr, cfg->addr_width);
    Nrf24_WriteReg(NRF_REG_SETUP_RETR,
                   (uint8_t)((uint8_t)(cfg->retr_delay << 4) | (cfg->retr_count & 0x0Fu)));
    Nrf24_WriteReg(NRF_REG_RF_CH, cfg->channel);

    uint8_t rf_setup = (uint8_t)((cfg->tx_power & 3u) << NRF_RF_SETUP_PWR_SHIFT);
    rf_setup |= cfg->rate_1mbps ? NRF_RF_SETUP_DR_LOW : NRF_RF_SETUP_DR_HIGH;
    Nrf24_WriteReg(NRF_REG_RF_SETUP, rf_setup);

    Nrf24_WriteReg(NRF_REG_RX_PW_P0, cfg->payload_width);
    Nrf24_WriteReg(NRF_REG_DYNPD, 0x00u);       /* 全静态负载，关闭动态包长 */
    Nrf24_WriteReg(NRF_REG_FEATURE, 0x00u);

    /* 残留态清零（IRQ 标志写 1 清零 + 双 FIFO 清空），Init 可重复执行 */
    Nrf24_WriteReg(NRF_REG_STATUS,
                   NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    Nrf24_FlushTx();
    Nrf24_FlushRx();

    /* 上电 + 模式位；PWR_UP→寄存器可用 ≥5ms（NRF_PWR_UP_DELAY_US） */
    uint8_t config_val = (uint8_t)(NRF_CONFIG_EN_CRC | NRF_CONFIG_CRCO |
                                   NRF_CONFIG_PWR_UP |
                                   (cfg->prim_rx ? NRF_CONFIG_PRIM_RX : 0u));
    Nrf24_WriteReg(NRF_REG_CONFIG, config_val);
    Bsp_DelayUs(NRF_PWR_UP_DELAY_US);

    /* 回读校验：任一不一致即报告链路/器件异常 */
    bool ok = true;
    ok = ok && (Nrf24_ReadReg(NRF_REG_CONFIG) == config_val);
    ok = ok && (Nrf24_ReadReg(NRF_REG_EN_AA) == (cfg->auto_ack ? 0x01u : 0x00u));
    ok = ok && (Nrf24_ReadReg(NRF_REG_SETUP_AW) == (uint8_t)(cfg->addr_width - 2u));
    ok = ok && (Nrf24_ReadReg(NRF_REG_RF_CH) == cfg->channel);
    ok = ok && (Nrf24_ReadReg(NRF_REG_RF_SETUP) == rf_setup);
    ok = ok && (Nrf24_ReadReg(NRF_REG_RX_PW_P0) == cfg->payload_width);

    uint8_t addr_back[NRF_ADDR_WIDTH_MAX];
    Nrf24_ReadRegBuf(NRF_REG_RX_ADDR_P0, addr_back, cfg->addr_width);
    ok = ok && (addr_back[0] == cfg->addr[0]);
    ok = ok && (addr_back[cfg->addr_width - 1u] == cfg->addr[cfg->addr_width - 1u]);

    return ok;
}
