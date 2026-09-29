/**
 * @file    nrf24_defs.h
 * @brief   nRF24L01+ 指令码 / 寄存器地址 / 位定义（纯常量，模块私有）
 *
 * 依赖 nrf24.c/h 的消费者（Modules/remote 与 PTX 测试工程）整组复制即可，
 * 本文件零外部依赖。数值依据 nRF24L01+ Product Specification v1.0。
 */
#ifndef NRF24_DEFS_H
#define NRF24_DEFS_H

/* ================================ 指令集 ================================== */

#define NRF_CMD_R_REGISTER      0x00u   /* 读寄存器：0x00|寄存器地址（5bit） */
#define NRF_CMD_W_REGISTER      0x20u   /* 写寄存器：0x20|寄存器地址（5bit） */
#define NRF_CMD_R_RX_PAYLOAD    0x61u   /* 读 RX 载荷（读后 FIFO 自动弹出） */
#define NRF_CMD_W_TX_PAYLOAD    0xA0u   /* 写 TX 载荷 */
#define NRF_CMD_FLUSH_TX        0xE1u
#define NRF_CMD_FLUSH_RX        0xE2u
#define NRF_CMD_NOP             0xFFu   /* 空操作：同时回读 STATUS */

/* ================================ 寄存器 ================================== */

#define NRF_REG_CONFIG          0x00u
#define NRF_REG_EN_AA           0x01u
#define NRF_REG_EN_RXADDR       0x02u
#define NRF_REG_SETUP_AW        0x03u
#define NRF_REG_SETUP_RETR      0x04u
#define NRF_REG_RF_CH           0x05u
#define NRF_REG_RF_SETUP        0x06u
#define NRF_REG_STATUS          0x07u
#define NRF_REG_OBSERVE_TX      0x08u   /* 重传/丢包计数（PTX 诊断） */
#define NRF_REG_RPD             0x09u   /* 载波检测 */
#define NRF_REG_RX_ADDR_P0      0x0Au
#define NRF_REG_TX_ADDR         0x10u
#define NRF_REG_RX_PW_P0        0x11u
#define NRF_REG_FIFO_STATUS     0x17u
#define NRF_REG_DYNPD           0x1Cu
#define NRF_REG_FEATURE         0x1Du

/* ============================== CONFIG 位域 =============================== */

#define NRF_CONFIG_PRIM_RX      0x01u   /* 1=PRX 0=PTX */
#define NRF_CONFIG_PWR_UP       0x02u
#define NRF_CONFIG_CRCO         0x04u   /* CRC 长度：1 = 2 字节 */
#define NRF_CONFIG_EN_CRC       0x08u

/* ============================== STATUS 位域 =============================== */
/* 三个中断标志均写 1 清零；上电复位值 STATUS = 0x0E（SPI 通断判据） */

#define NRF_STATUS_RX_DR        0x40u
#define NRF_STATUS_TX_DS        0x20u
#define NRF_STATUS_MAX_RT       0x10u

/* ============================ FIFO_STATUS 位域 ============================ */

#define NRF_FIFO_RX_EMPTY       0x01u
#define NRF_FIFO_TX_EMPTY       0x20u

/* ============================== RF_SETUP 位域 ============================= */

#define NRF_RF_SETUP_CONT_WAVE  0x80u
#define NRF_RF_SETUP_DR_LOW     0x20u   /* 1Mbps（RF_DR_LOW=1, RF_DR_HIGH=0） */
#define NRF_RF_SETUP_DR_HIGH    0x08u   /* 2Mbps（RF_DR_LOW=0, RF_DR_HIGH=1） */
#define NRF_RF_SETUP_PWR_SHIFT  1u      /* RF_PWR 位于 bit2:1，0..3 = -18/-12/-6/0 dBm */
#define NRF_RF_SETUP_PWR_MASK   0x06u

/* ================================ 硬件极限 ================================ */

#define NRF_ADDR_WIDTH_MAX      5u      /* 地址宽 3~5 字节 */
#define NRF_PAYLOAD_MAX         32u     /* 单包载荷上限 */
#define NRF_CHANNEL_MAX         125u    /* RF 通道上限（2.400~2.525GHz） */
#define NRF_PWR_UP_DELAY_US     5000u   /* PWR_UP→寄存器可用，手册 Tpd2pwr 1.5ms 取保守 5ms */
#define NRF_CE_PULSE_US         15u     /* PTX 触发发送的 CE 高脉宽，手册最小 10us 取余量 */

#endif /* NRF24_DEFS_H */
