/**
 * @file    bsp_vofa.h
 * @brief   VOFA+ JustFloat 引擎发送通道（UART4，专用于整定高速数据流）
 *
 * JustFloat 帧格式（vofa.plus 官方 docs/learning/dataengines/justfloat）：
 *   N 个 float32（小端）+ 帧尾 0x00 0x00 0x80 0x7F（float +Inf）。
 *   VOFA+ 按帧尾切帧，通道数在协议设置里填 N。
 *
 * 通道归属：日志走 USART2（bsp_log），本模块独占 UART4（PC10/PC11），
 * 运行时把波特率提到 ROBOT_VOFA_BAUD（921600）——1kHz×20B 的 IT 发送
 * 占空比 ~22%，不挤占控制环。上层（测试台）控制发送时机与频率，
 * 本模块只管打包与发出去。
 */
#ifndef BSP_VOFA_H
#define BSP_VOFA_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief  初始化：UART4 重配为 ROBOT_VOFA_BAUD（幂等，可重复调用）
 * @note   MX_UART4_Init 已完成引脚/时序配置，这里只改波特率；
 *         须在 MX_UART4_Init 之后调用（main 初始化序保证）
 */
void Bsp_Vofa_Init(void);

/**
 * @brief  BRR 反算的实际波特率（固件自证用；APB1=42MHz 前提）
 */
uint32_t Bsp_Vofa_GetBaud(void);

/**
 * @brief  发送一帧 JustFloat（N 通道 float + 帧尾）
 * @param  vals  float 数组（通道顺序 = VOFA+ 通道顺序）
 * @param  n     通道数，1..ROBOT_VOFA_MAX_CH；超界丢弃
 * @retval true=已提交发送 false=上一帧还在发、本帧丢弃
 * @note   中断发送 + 忙丢弃：宁丢帧不阻塞——控制环节拍优先于示波器数据；
 *         上层以 1kHz 调用、波特率按 ROBOT_VOFA_BAUD 配，正常不丢
 */
bool Bsp_Vofa_SendFloats(const float *vals, uint8_t n);

/* ---- 命令接收（同一根 UART4 双向；VOFA+ 控件/串口助手发文本行） ---- */

/**
 * @brief  初始化后调用一次：挂 UART4 单字节接收中断（内部自动续挂）
 * @note   已在 Bsp_Vofa_Init 尾部完成，一般无需再调
 */
void Bsp_Vofa_RxArm(void);

/**
 * @brief  RX 中断喂入口（bsp_log.c 的 HAL_UART_RxCpltCallback 在 UART4 分支
 *         调用——回调全仓库唯一，本模块不另占弱符号；字节取自本模块缓冲）
 */
void Bsp_Vofa_OnRxIrq(void);

/**
 * @brief  非阻塞取一条完整命令行（\r 或 \n 结尾，不含行尾符）
 * @retval true=取到（out 以 \0 结尾） false=暂无
 * @note   上一行未取走时新行丢弃（整定命令低频，正常不丢）
 */
bool Bsp_Vofa_ReadLine(char *out, uint8_t maxlen);

#endif /* BSP_VOFA_H */
