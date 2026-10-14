/**
 * @file    bsp_adc.h
 * @brief   摇杆双通道 ADC 读取（PA1=X / PA2=Y，12 位右对齐 0..4095）
 *
 * 设计边界：ADC1 单实例、编译期固定，模块级函数不做框架（实例化边界三问）。
 * 采样策略沿用车端 F103 时代 Adc3_Read 先例：单 rank 逐次读，每次读前重配通道
 * （F1 扫描模式无 DMA 时 DR 会被 rank2 覆盖，逐通道单转换最稳）；
 * 摇杆电位器典型 10kΩ，71.5 周期采样 @12MHz ADC 时钟 ≈ 7µs，源阻抗余量充足。
 */
#ifndef REMOTER_BSP_ADC_H
#define REMOTER_BSP_ADC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  校验 hadc1 就绪（Instance==ADC1 且状态 READY），诊断用
 */
bool Adc_IsReady(void);

/**
 * @brief  读摇杆 X/Y 一对原始值（阻塞，两次单转换 + 轮询，单次 <20µs）
 * @param  x_raw / y_raw 输出 0..4095，NULL 则跳过该轴
 * @retval false = 参数全空 / hadc1 未就绪 / 轮询超时（见 ROBOT_ADC_TIMEOUT_MS）
 * @note   非并发契约：仅遥控发送任务一个消费者。
 */
bool Adc_JoyRead(uint16_t *x_raw, uint16_t *y_raw);

#endif /* REMOTER_BSP_ADC_H */
