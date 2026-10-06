/**
 * @file    bsp_log.h
 * @brief   调试日志：USART2（PD5/PA3）纯中断收发 + 环形缓冲 + printf 重定向
 *
 * 规格见《BSP层开发规划》四.2 / 五 / 七.2：
 *  - TX：Log_Printf 格式化入队，TxCpltCallback 链式推进，队列满丢弃并计数；
 *  - RX：单字节中断搬入环形缓冲，Log_Poll 由任务侧取出；ISR 只搬字节，
 *    无 printf / malloc / 阻塞，符合优先级 5 约束；
 *  - 全部静态缓冲，无动态分配；临界区用短关中断，不依赖 RTOS API。
 */
#ifndef BSP_LOG_H
#define BSP_LOG_H

#include <stdint.h>

void     Log_Init(void);
void     Log_Printf(const char *fmt, ...);
uint16_t Log_Poll(uint8_t *buf, uint16_t len);  /* 从 RX 环形缓冲取出，返回实际字节数 */

#endif //BSP_LOG_H
