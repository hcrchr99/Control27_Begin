/**
 * @file    nrf24.h
 * @brief   nRF24L01+ 寄存器级驱动（器件知识层，仅依赖 bsp_spi/bsp_sys）
 *
 * 消费者有两个：Modules/remote（本仓库链路层）与 PTX 测试工程（最小系统板，
 * 复制 nrf24_defs.h + nrf24.c/h + bsp 三件套即可独立编译）。因此本文件
 * 不依赖 robot_config.h / bsp_pin.h——所有可调数值经 Nrf24_Config_t 传入。
 * CSN/CE 时序由本层经 Spi_Csn/Spi_Ce 管理，调用方不碰电平。
 */
#ifndef NRF24_H
#define NRF24_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf24_defs.h"

typedef struct
{
    bool    prim_rx;                        /* true=PRX（收），false=PTX（发） */
    uint8_t channel;                        /* RF 通道 0..125（2.400GHz+n MHz） */
    bool    rate_1mbps;                     /* true=1Mbps（距离优先），false=2Mbps */
    uint8_t tx_power;                       /* RF_PWR 0..3 = -18/-12/-6/0 dBm */
    uint8_t addr[NRF_ADDR_WIDTH_MAX];       /* 地址，写入按字节序原样 */
    uint8_t addr_width;                     /* 3..5 */
    uint8_t payload_width;                  /* 静态负载宽 1..32（收发两端须一致） */
    bool    auto_ack;                       /* 管道 0 自动应答（EN_AA） */
    uint8_t retr_delay;                     /* 重传等待 0..15（×250us，仅 PTX 有效） */
    uint8_t retr_count;                     /* 自动重传上限 0..15（仅 PTX 有效） */
} Nrf24_Config_t;

/* ---- 寄存器访问（内部管理 CSN 时序） ---- */
uint8_t Nrf24_ReadReg(uint8_t reg);
void    Nrf24_ReadRegBuf(uint8_t reg, uint8_t *buf, uint8_t len);
void    Nrf24_WriteReg(uint8_t reg, uint8_t val);
void    Nrf24_WriteRegBuf(uint8_t reg, const uint8_t *buf, uint8_t len);
uint8_t Nrf24_GetStatus(void);
void    Nrf24_FlushTx(void);
void    Nrf24_FlushRx(void);

/**
 * @brief  按配置写全组寄存器并逐项回读校验（初始化即验证 SPI 链路）。
 *         内部完成 PWR_UP=0 → 配置 → PWR_UP=1（含 ≥5ms 上电延时）→ IRQ/FIFO 清零。
 *         末态：待机（CE 低），是否进入收发由调用方控制 CE。
 * @retval false = 参数非法或任一回读不一致（总线/器件异常）
 */
bool Nrf24_Configure(const Nrf24_Config_t *cfg);

#endif /* NRF24_H */
