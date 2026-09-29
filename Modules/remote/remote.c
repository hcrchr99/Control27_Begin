/**
 * @file    remote.c
 * @brief   无线遥控链路层实现（PRX 收包队列 / PTX 阻塞发送 / 链路看门狗）
 */
#include "remote.h"
#include "nrf24.h"
#include "bsp_spi.h"
#include "bsp_gpio.h"
#include "bsp_pin.h"
#include "bsp_sys.h"
#include "robot_config.h"
#include "cmsis_os.h"
#include <string.h>

/* ================================ 内部常量 ================================ */

#define SIG_REMOTE_IRQ      0x0001u     /* 任务信号位：IRQ 事件（osSignalSet 置位） */
#define SERVICE_TIMEOUT_MS  20u         /* 信号等待兜底：RX FIFO 3 级深，20ms 轮询
                                         * 在实际遥控帧率（≤500Hz）下不会溢出 */

/* ================================ 静态状态 ================================ */

static RemoteMode_t s_mode;
static osThreadId   s_task;             /* IRQ 通知目标 = Remote_Init 调用者 */
static uint8_t      s_payload;          /* 静态负载宽（= ROBOT_REMOTE_PAYLOAD） */

/* PRX 帧环形队列：生产者（Remote_Service 搬运 FIFO）与消费者（Remote_ReadPacket）
 * 都在同一个服务任务内，读写点串行，无需锁/临界区 */
static uint8_t      s_rxq[ROBOT_REMOTE_RX_QUEUE][ROBOT_REMOTE_PAYLOAD];
static uint8_t      s_rx_head;          /* 写入位 */
static uint8_t      s_rx_tail;          /* 读出位；s_rx_head == s_rx_tail 即空 */

static volatile uint32_t s_last_rx_ms;  /* 最近一帧到达时刻（Bsp_GetMs 域） */
static volatile bool     s_ever_rx;     /* 上电以来是否收到过帧（看门狗前置条件） */
static uint32_t     s_rx_count;         /* 累计收到帧数（含队列覆盖丢弃的） */
static uint32_t     s_tx_fail;          /* PTX 累计发送失败（MAX_RT/超时） */

/* ============================== IRQ 回调（ISR） =========================== */

/* bsp_gpio 回调契约：ISR 上下文只允许置标志 / FromISR 操作。
 * osSignalSet 在 CMSIS_V1 明确 ISR 安全，此处一行完成事件投递。 */
static void Remote_IrqCallback(void)
{
    if (s_task != NULL)
    {
        (void)osSignalSet(s_task, (int32_t)SIG_REMOTE_IRQ);
    }
}

/* ================================ PRX 收包 ================================ */

/* 把 RX FIFO 搬空入队。每帧先写 1 清 RX_DR，再按 FIFO_STATUS.RX_EMPTY 判空，
 * 不依赖 RX_DR 电平语义（连续收包时 RX_DR 行为不作保证） */
static void DrainRxFifo(void)
{
    Nrf24_WriteReg(NRF_REG_STATUS, NRF_STATUS_RX_DR);

    for (;;)
    {
        if ((Nrf24_ReadReg(NRF_REG_FIFO_STATUS) & NRF_FIFO_RX_EMPTY) != 0u)
        {
            break;
        }

        uint8_t frame[NRF_PAYLOAD_MAX];
        Nrf24_ReadRegBuf(NRF_CMD_R_RX_PAYLOAD, frame, s_payload);

        s_rx_count++;
        s_last_rx_ms = Bsp_GetMs();
        s_ever_rx = true;

        uint8_t next = (uint8_t)((s_rx_head + 1u) % ROBOT_REMOTE_RX_QUEUE);
        if (next == s_rx_tail)              /* 队列满：覆盖最旧帧，保最新指令 */
        {
            s_rx_tail = (uint8_t)((s_rx_tail + 1u) % ROBOT_REMOTE_RX_QUEUE);
        }
        memcpy(s_rxq[s_rx_head], frame, s_payload);
        s_rx_head = next;
    }
}

/* ================================ 公开接口 ================================ */

bool Remote_Init(RemoteMode_t mode)
{
    if (!Spi_Init())
    {
        return false;
    }

    /* SPI 通断机内预检：nRF24 上电复位 STATUS = 0x0E；全 0x00/0xFF 均为
     * 总线异常（MISO 悬空/未共地/CSN 时序错） */
    uint8_t st = Nrf24_GetStatus();
    if (st == 0x00u || st == 0xFFu)
    {
        return false;
    }

    Nrf24_Config_t cfg = {
        .prim_rx       = (mode == REMOTE_MODE_PRX),
        .channel       = ROBOT_RF_CHANNEL,
        .rate_1mbps    = (ROBOT_RF_DATA_RATE == 1),
        .tx_power      = ROBOT_RF_TX_POWER,
        .addr          = ROBOT_RF_ADDR_BYTES,
        .addr_width    = ROBOT_RF_ADDR_WIDTH,
        .payload_width = ROBOT_REMOTE_PAYLOAD,
        .auto_ack      = true,
        .retr_delay    = ROBOT_RF_RETR_DELAY,
        .retr_count    = ROBOT_RF_RETR_COUNT,
    };
    if (!Nrf24_Configure(&cfg))
    {
        return false;
    }

    s_mode    = mode;
    s_task    = osThreadGetId();    /* 谁 Init 谁服务（remote.h 契约） */
    s_payload = ROBOT_REMOTE_PAYLOAD;
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_rx_count = 0u;
    s_tx_fail = 0u;
    s_ever_rx = false;
    s_last_rx_ms = 0u;

    /* 行为注入走 bsp_gpio（规则 1）；注册在配置完成后，避免配置期伪中断 */
    if (!Exti_Attach(PIN_WL_IRQ_GPIO_PIN, Remote_IrqCallback))
    {
        return false;
    }

    if (mode == REMOTE_MODE_PRX)
    {
        Spi_Ce(true);   /* 持续使能进入 RX（CE 高 >130us 生效，此后常高） */
    }
    return true;
}

void Remote_Service(void)
{
    if (s_mode != REMOTE_MODE_PRX)
    {
        /* PTX：发包在 Remote_SendPacket 内同步等待 IRQ，此处仅让出 CPU */
        osDelay(SERVICE_TIMEOUT_MS);
        return;
    }

    /* 等 IRQ；20ms 兜底唤醒用于覆盖"信号偶发丢失/初始化前残留标志"等边界 */
    (void)osSignalWait((int32_t)SIG_REMOTE_IRQ, SERVICE_TIMEOUT_MS);
    DrainRxFifo();
}

bool Remote_ReadPacket(uint8_t *buf, uint8_t size)
{
    if (buf == NULL || size < ROBOT_REMOTE_PAYLOAD || s_rx_head == s_rx_tail)
    {
        return false;
    }
    memcpy(buf, s_rxq[s_rx_tail], ROBOT_REMOTE_PAYLOAD);
    s_rx_tail = (uint8_t)((s_rx_tail + 1u) % ROBOT_REMOTE_RX_QUEUE);
    return true;
}

bool Remote_SendPacket(const uint8_t *buf, uint8_t size)
{
    if (s_mode != REMOTE_MODE_PTX || buf == NULL || size > ROBOT_REMOTE_PAYLOAD)
    {
        return false;
    }

    /* 短帧补零到静态负载宽（收发两端 RX_PW 必须一致） */
    uint8_t frame[ROBOT_REMOTE_PAYLOAD] = {0};
    memcpy(frame, buf, size);

    /* 清历史 IRQ 标志，防上一包事件污染本次等待 */
    Nrf24_WriteReg(NRF_REG_STATUS, NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    Nrf24_FlushTx();
    Nrf24_WriteRegBuf(NRF_CMD_W_TX_PAYLOAD, frame, s_payload);

    /* CE 脉冲 ≥10us 触发一次发送（NRF_CE_PULSE_US 取 15us 余量） */
    Spi_Ce(true);
    Bsp_DelayUs(NRF_CE_PULSE_US);
    Spi_Ce(false);

    /* IRQ（TX_DS 或 MAX_RT）到达即醒；超时兜底 ROBOT_REMOTE_TX_TIMEOUT_MS
     * （重传耗尽最坏 = retr_count × retr_delay，50ms 覆盖 10×3.75ms） */
    (void)osSignalWait((int32_t)SIG_REMOTE_IRQ, ROBOT_REMOTE_TX_TIMEOUT_MS);

    uint8_t st = Nrf24_GetStatus();
    Nrf24_WriteReg(NRF_REG_STATUS, (uint8_t)(st & (NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT)));
    if ((st & NRF_STATUS_TX_DS) != 0u)
    {
        return true;
    }

    /* MAX_RT 后 TX FIFO 保留原包（手册行为），必须清空防止连环重发 */
    Nrf24_FlushTx();
    s_tx_fail++;
    return false;
}

bool Remote_IsLinkUp(void)
{
    return s_ever_rx &&
           ((Bsp_GetMs() - s_last_rx_ms) < ROBOT_REMOTE_WATCHDOG_MS);
}

uint32_t Remote_GetRxCount(void)
{
    return s_rx_count;
}

uint32_t Remote_GetTxFailCount(void)
{
    return s_tx_fail;
}
