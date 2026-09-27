/**
 * @file    bsp_log.c
 * @brief   UART4 IT 环形缓冲日志实现
 */
#include "bsp_log.h"
#include "bsp_pin.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ================================ 配置 ==================================== */

#define LOG_RX_SIZE     256u    /* RX 环形缓冲（规划资源预算 0.5KB 的一部分） */
#define LOG_TX_SIZE     512u    /* TX 环形缓冲：任务入队快、串口出队 115.2B/ms，留余量 */
#define LOG_FMT_SIZE    128u    /* 单条格式化上限，超出截断 */
#define LOG_CHUNK_SIZE  64u     /* 每次 Transmit_IT 的线性块大小 */

/* ================================ 数据 ==================================== */

static uint8_t  log_rx_buf[LOG_RX_SIZE];
static volatile uint16_t log_rx_head;
static volatile uint16_t log_rx_tail;

static uint8_t  log_tx_buf[LOG_TX_SIZE];
static volatile uint16_t log_tx_head;
static volatile uint16_t log_tx_tail;
static volatile uint8_t  log_tx_busy;

static uint8_t  log_tx_chunk[LOG_CHUNK_SIZE];
static uint8_t  log_rx_byte;    /* HAL_UART_Receive_IT 单字节落点 */

static uint32_t log_tx_drop_cnt;

/* ================================ 内部工具 ================================ */

static uint16_t ring_count(uint16_t size, volatile uint16_t head, volatile uint16_t tail)
{
    return (uint16_t)((head - tail) % size);
}

/* PRIMASK 保存/恢复，保证在关中断调用者内也可安全嵌套 */
static uint32_t log_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void log_unlock(uint32_t primask)
{
    if (!primask)
    {
        __enable_irq();
    }
}

/* 把 TX 环形缓冲弹出一个线性块交给 UART，须在关中断（或 ISR）内调用 */
static void log_tx_kick_locked(void)
{
    if (log_tx_busy)
    {
        return;
    }
    uint16_t cnt = ring_count(LOG_TX_SIZE, log_tx_head, log_tx_tail);
    if (cnt == 0u)
    {
        return;
    }
    if (cnt > LOG_CHUNK_SIZE)
    {
        cnt = LOG_CHUNK_SIZE;
    }
    for (uint16_t i = 0; i < cnt; i++)
    {
        log_tx_chunk[i] = log_tx_buf[log_tx_tail];
        log_tx_tail = (uint16_t)((log_tx_tail + 1u) % LOG_TX_SIZE);
    }
    log_tx_busy = 1u;
    (void)HAL_UART_Transmit_IT((UART_HandleTypeDef *)PIN_LOG_UART, log_tx_chunk, cnt);
}

/* 压入 TX 环形缓冲，满则丢弃本次剩余字节并计数 */
static void log_tx_push(const uint8_t *data, uint16_t len)
{
    uint32_t primask = log_lock();

    for (uint16_t i = 0; i < len; i++)
    {
        if (ring_count(LOG_TX_SIZE, log_tx_head, log_tx_tail) >= LOG_TX_SIZE - 1u)
        {
            log_tx_drop_cnt++;
            break;
        }
        log_tx_buf[log_tx_head] = data[i];
        log_tx_head = (uint16_t)((log_tx_head + 1u) % LOG_TX_SIZE);
    }
    log_tx_kick_locked();

    log_unlock(primask);
}

/* ================================ 对外 API ================================ */

void Log_Init(void)
{
    log_rx_head = log_rx_tail = 0u;
    log_tx_head = log_tx_tail = 0u;
    log_tx_busy = 0u;
    log_tx_drop_cnt = 0u;

    /* 启动 RX 单字节中断链；UART4 的 MspInit 已使能 NVIC(优先级 5,0) */
    (void)HAL_UART_Receive_IT((UART_HandleTypeDef *)PIN_LOG_UART, &log_rx_byte, 1u);
}

void Log_Printf(const char *fmt, ...)
{
    char    line[LOG_FMT_SIZE];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    if (n <= 0)
    {
        return;
    }
    if ((uint32_t)n >= sizeof(line))
    {
        n = (int)sizeof(line) - 1;  /* vsnprintf 截断时返回的是"应有长度"，需钳制 */
    }
    log_tx_push((const uint8_t *)line, (uint16_t)n);
}

uint16_t Log_Poll(uint8_t *buf, uint16_t len)
{
    uint32_t primask = log_lock();

    uint16_t cnt = ring_count(LOG_RX_SIZE, log_rx_head, log_rx_tail);
    if (cnt > len)
    {
        cnt = len;
    }
    for (uint16_t i = 0; i < cnt; i++)
    {
        buf[i] = log_rx_buf[log_rx_tail];
        log_rx_tail = (uint16_t)((log_rx_tail + 1u) % LOG_RX_SIZE);
    }

    log_unlock(primask);
    return cnt;
}

/* ======================== printf / putchar 重定向 ========================= */

/* newlib 的 printf 最终走 _write（newlib-nano 同样），Keil microlib 风格的
 * fputc 重定向对 GCC 工具链无效，两者都接进 TX 队列以兼容手写 putchar。 */
__attribute__((used)) int _write(int fd, char *ptr, int len)
{
    (void)fd;
    if (len <= 0)
    {
        return 0;
    }
    log_tx_push((const uint8_t *)ptr, (uint16_t)len);
    return len;
}

#ifdef __GNUC__
__attribute__((used)) int fputc(int ch, FILE *f)
{
    (void)f;
    uint8_t b = (uint8_t)ch;
    log_tx_push(&b, 1u);
    return ch;
}
#endif

/* ============================ HAL 回调（全局唯一） ========================= */

/* 全仓库回调占用情况（2026-09-27 核对）：UART 回调仅本文件定义；
 * HAL_TIM_PeriodElapsedCallback 已被 main.c 占用（TIM6 时基），勿在此重定义。 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == PIN_LOG_UART)
    {
        log_tx_busy = 0u;
        log_tx_kick_locked();   /* ISR 上下文，本身就在"关中断"语义内 */
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == PIN_LOG_UART)
    {
        log_rx_buf[log_rx_head] = log_rx_byte;
        log_rx_head = (uint16_t)((log_rx_head + 1u) % LOG_RX_SIZE);
        (void)HAL_UART_Receive_IT(huart, &log_rx_byte, 1u);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == PIN_LOG_UART)
    {
        /* F1 HAL 语义（stm32f1xx_hal_uart.c IRQHandler）：ORE 为阻断错误，
         * 已先 EndRxTransfer（RxState=READY）再进本回调；NE/FE 不中止传输。
         * 故仅在 READY 时清 ORE 并重新挂接收，BUSY_RX 时不可重复挂。 */
        if (huart->RxState == HAL_UART_STATE_READY)
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            (void)HAL_UART_Receive_IT(huart, &log_rx_byte, 1u);
        }
    }
}
