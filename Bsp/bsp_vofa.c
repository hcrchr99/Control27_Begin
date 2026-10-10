/**
 * @file    bsp_vofa.c
 * @brief   VOFA+ JustFloat 发送通道实现（UART4 @ ROBOT_VOFA_BAUD）
 *
 * 实现要点：
 *  - 波特率运行时改：MX_UART4_Init 固定 115200（CubeMX 源），这里 DeInit 后
 *    改 huart4.Init.BaudRate 再 Init——CubeMX 重新生成不回退此改动；
 *  - 单缓冲 + 忙丢弃：HAL_UART_Transmit_IT 期间 gState=BUSY_TX，此时直接
 *    丢帧返回——缓冲只在发送完成后才允许被下一帧复用，无需双缓冲；
 *  - 4 字节帧尾 = float 1 的指数部分全 1（+Inf），正常数据不可能出现，
 *    即帧分隔符（JustFloat 协议约定）。
 */
#include "bsp_vofa.h"
#include <string.h>
#include "usart.h"
#include "robot_config.h"

#define VOFA_TAIL_SIZE  4u
#define VOFA_FRAME_SIZE (ROBOT_VOFA_MAX_CH * 4u + VOFA_TAIL_SIZE)

static uint8_t s_frame[VOFA_FRAME_SIZE];

void Bsp_Vofa_Init(void)
{
    if (huart4.Init.BaudRate != ROBOT_VOFA_BAUD)
    {
        HAL_UART_DeInit(&huart4);
        huart4.Init.BaudRate = ROBOT_VOFA_BAUD;
        if (HAL_UART_Init(&huart4) != HAL_OK)
        {
            Error_Handler();
        }
    }
    Bsp_Vofa_RxArm();                   /* 命令行接收入口一并挂好 */
}

bool Bsp_Vofa_SendFloats(const float *vals, uint8_t n)
{
    if (n == 0u || n > ROBOT_VOFA_MAX_CH)
    {
        return false;
    }
    if (huart4.gState != HAL_UART_STATE_READY)
    {
        return false;                   /* 上一帧还在发：丢帧不阻塞 */
    }

    uint8_t *p = s_frame;
    for (uint8_t i = 0u; i < n; i++)
    {
        uint32_t raw;                   /* float 位型拷贝（F4 小端，直接按字节铺） */
        (void)memcpy(&raw, &vals[i], 4u);
        *p++ = (uint8_t)(raw);
        *p++ = (uint8_t)(raw >> 8);
        *p++ = (uint8_t)(raw >> 16);
        *p++ = (uint8_t)(raw >> 24);
    }
    *p++ = 0x00u;                       /* 帧尾 00 00 80 7F = +Inf */
    *p++ = 0x00u;
    *p++ = 0x80u;
    *p   = 0x7Fu;

    return (HAL_UART_Transmit_IT(&huart4, s_frame, (uint16_t)(n * 4u + VOFA_TAIL_SIZE)) == HAL_OK);
}

/* ---- 命令接收：单字节 IT 攒行，\r/\n 结尾；行未取走时新行整行丢弃 ---- */

static uint8_t s_rx_byte;
static char     s_line[ROBOT_VOFA_RX_LINE];
static uint8_t  s_line_len;
static volatile bool s_line_ready;
static uint16_t s_line_dropped;         /* 丢弃行计数（诊断用） */

void Bsp_Vofa_RxArm(void)
{
    (void)HAL_UART_Receive_IT(&huart4, &s_rx_byte, 1u);
}

/* BRR 反算实际波特率：BRR 寄存器值 = USARTDIV×16，baud = fCK ÷ BRR值
 * （如 921600@42MHz → BRR=45 → 933333，+1.3% 为 HAL 舍入，合法容差内） */
uint32_t Bsp_Vofa_GetBaud(void)
{
    uint32_t brr = UART4->BRR;
    return (brr > 0u) ? 42000000u / brr : 0u;
}

void Bsp_Vofa_OnRxIrq(void)
{
    /* 字节从本模块自己的接收缓冲取——HAL 在中断里已把 DR 搬进 s_rx_byte，
     * 调用方（bsp_log 回调）不要代传别的缓冲（曾传成 USART2 的变量顶包，
     * 命令全哑：硬件收到、软件拼错） */
    uint8_t byte = s_rx_byte;

    if (byte == '\r' || byte == '\n')
    {
        if (s_line_len > 0u)
        {
            if (!s_line_ready)
            {
                s_line[s_line_len] = '\0';
                s_line_ready = true;
            }
            else
            {
                s_line_dropped++;       /* 上一行还没被取走 */
            }
            s_line_len = 0u;
        }
    }
    else if (!s_line_ready && s_line_len < (uint8_t)(sizeof(s_line) - 1u))
    {
        s_line[s_line_len++] = (char)byte;
    }
    /* 行缓冲满/未消费时来的普通字节直接丢，等行尾重新对齐 */
    Bsp_Vofa_RxArm();
}

bool Bsp_Vofa_ReadLine(char *out, uint8_t maxlen)
{
    /* 拷贝-清零段与 OnRxIrq 共享 s_line/s_line_len/s_line_ready，整段关中断
     * 原子化（W2.3 复盘遗留修复）：旧序"ready=false 先落、len 后清"的窗口里
     * ISR 可携新行字节按旧 len 续写，首字节落进上一行残尾造成整行错位。
     * 拷贝 ≤24 字节约 1µs@168MHz，UART 接收中断推迟无感（硬件挂起不丢字节）。
     * 契约：仅任务侧调用（ISR 内调用会裸开中断）。 */
    __disable_irq();
    if (!s_line_ready)
    {
        __enable_irq();
        return false;
    }
    uint8_t i = 0u;
    for (; (i < (uint8_t)(maxlen - 1u)) && s_line[i] != '\0'; i++)
    {
        out[i] = s_line[i];
    }
    out[i] = '\0';
    s_line_len = 0u;
    s_line_ready = false;
    __enable_irq();
    return true;
}
