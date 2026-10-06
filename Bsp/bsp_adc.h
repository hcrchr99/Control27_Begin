/**
 * @file    bsp_adc.h
 * @brief   ADC1/2 双同步规则组 DMA 循环采样（双 rank：电源 V/I + 关节 J0/J1）
 *
 * 设计（BSP层开发规划 四.4 / ADC方案B配置指南；2026-10-05 随 F407VG 迁移勘误
 * + 双 rank 重构）：
 *  - CubeMX 基线（方案 B 演进形，零中断）：ADC1(主)+ADC2(从) 双同步 REGSIMULT、
 *    双 rank 等长序列（rank1: IN4+IN5=电源 V/I，rank2: IN12+IN13=关节 J0/J1）、
 *    ADC1 连续转换、DMA2_Stream0 循环搬运（Word 对齐，一次 Word = 一对同步样本，
 *    字流按 rank 交替：偶=rank1 对、奇=rank2 对），DMA 中断已在 main.c 屏蔽，
 *    数据由 DMA 持续刷新，任务直接读内存；
 *  - ⚠ 双同步启动必须用 HAL_ADCEx_MultiModeStart_DMA（HAL_ADC_Start_DMA 在
 *    多模式下直接返回 HAL_ERROR，F1/F4 HAL 同此约束）；
 *  - ⚠ F4 前提（.ioc 源头保证，勿在运行期补救）：ADC1 的
 *    DMAContinuousRequests=ENABLE（F4 MultiModeStart_DMA 按它置 CCR.DDS，
 *    DISABLE 则循环 DMA 搬完一窗即停）；采样时间 84CYCLES（F4 无 55.5 档）；
 *  - ⚠ F1 遗留项/架构差异（勘误#5/#6 速查，全文见 docs/BSP调试日志.md）：
 *    F4 无 ADC 校准；F4 HAL 只使能多模式主机（从机 ADON 由 Adc_Init 补位）；
 *    MULTI≠0 屏蔽 ADC3 独立启动 → ADC3 退役、J0/J1 并入 rank2——J 采样
 *    从"按需轮询"升级为"连续采样"（对 W2.4 堵转判据更友好）；
 *  - 只出外设级 API（原始码/均值），物理量换算在 Modules/power（标定点唯一）；
 *  - 缓冲与窗口样本数 = ROBOT_POWER_WINDOW（robot_config.h，编译期定死）。
 */
#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>
#include <stdbool.h>

/* 双同步原始对：多模式一次 32 位搬运读 CDR，低 16=ADC1、高 16=ADC2。
 * rank1 对 = (IN4 电流, IN5 电压)，rank2 对 = (IN12 J0, IN13 J1)
 * （采样对象归属见 bsp_pin.h） */
typedef struct
{
    uint16_t raw_i;     /* ADC1 IN4 原始码（0..4095） */
    uint16_t raw_v;     /* ADC2 IN5 原始码（0..4095） */
} AdcPair_t;

/* 关节通道序号（Adc_J_Read 的 ch 参数） */
#define ADC_J_CH1     1u     /* rank2 低半字 = ADC1_IN12（PC2，J0） */
#define ADC_J_CH2     2u     /* rank2 高半字 = ADC2_IN13（PC3，J1） */

/**
 * @brief  启动采样（幂等，重复调用无副作用）。
 *         手动上电从机（勘误#5）→ 双同步 MultiModeStart_DMA 循环搬运 →
 *         丢前 2 轮首批样本（约 300µs 忙等，之后缓冲全量为新鲜数据）。
 *         不重跑 MX_ADCx_Init（main.c 已在调度器启动前调用）。
 * @note   需 Bsp_Init/Log_Init 已跑（main.c USER CODE 2 中按序调用）。
 */
void Adc_Init(void);

/**
 * @brief  取最新一对同步 V/I（rank1，偶下标；uint32 读原子无撕裂）。
 * @param  out 接收原始对，不可为 NULL
 * @retval false = out 为 NULL / 未初始化 / 首轮 rank1 对未落
 */
bool Adc_GetLatest(AdcPair_t *out);

/**
 * @brief  取全窗均值（窗长 = ROBOT_POWER_WINDOW 对，偶下标）。
 *         逐对累加：32 位对读原子，各对之间可能跨 DMA 轮次新旧混合——对均值
 *         影响可忽略（方案 B 指南 §4.2 简化做法，PRIMASK 关中断本就拦不住
 *         DMA 硬件写入，不做伪快照）。
 * @retval false = out 为 NULL / 未初始化 / 首轮未填满整环
 */
bool Adc_GetAvg(AdcPair_t *out);

/**
 * @brief  取最新一对关节原始码（rank2，奇下标；连续采样，非阻塞无超时路径）。
 * @param  ch ADC_J_CH1 / ADC_J_CH2
 * @param  raw 接收原始码，不可为 NULL
 * @retval false = 参数非法 / 未初始化 / 首轮 J 对未落
 * @note   最新 rank1 为偶位时本轮 rank2 尚未落，返回上一轮的（滞后约
 *         9.2µs，20~50ms 级调用方无感）。相比 F1 的按需轮询：无阻塞、
 *         无 2ms 超时路径、失败源只剩"还没采到过"。
 */
bool Adc_J_Read(uint8_t ch, uint16_t *raw);

/**
 * @brief  诊断：双同步 DMA 采样是否已启动（Adc_Init 跑过且 HAL 报 OK）
 */
bool Adc_IsReady(void);

#endif /* BSP_ADC_H */
