/**
 * @file    bsp_adc.h
 * @brief   ADC1/2 双同步规则组 DMA 循环采样 + ADC3 独立扫描轮询
 *
 * 设计（BSP层开发规划 四.4 / ADC方案B配置指南，2026-10-03 预研勘误）：
 *  - CubeMX 基线（方案 B，零中断）：ADC1(主)+ADC2(从) 双同步 REGSIMULT、
 *    ADC1 连续转换、DMA1_Ch1 循环搬运（Word 对齐，一次 Word = 一对同步样本），
 *    DMA 中断已在 main.c 屏蔽，数据由 DMA 持续刷新，任务直接读内存；
 *  - ⚠ 双同步启动必须用 HAL_ADCEx_MultiModeStart_DMA（HAL_ADC_Start_DMA 在
 *    多模式下直接返回 HAL_ERROR，stm32f1xx_hal_adc.c 的 multimode 分支）；
 *  - ⚠ 从机 ADC2 两个缺失配置（HAL_ADC_Init 有意把 EXTTRIG 留给 Start_xxx、
 *    CubeMX 给从机生成 CONT=DISABLE，而 MultiModeStart_DMA 只管主机）——
 *    Adc_Init 内重跑 HAL_ADC_Init(CONT=ENABLE) + SET_BIT(EXTTRIG)，否则从机
 *    只被触发一拍即停、DR 高半字冻结（板上实测 2026-10-06 两轮破案）；
 *  - ⚠ F103 的 ADC CR2 无 DDS 位（"DMA Continuous Requests" 为 F4/L4 系概念），
 *    方案 B 指南 §2.1/§3 该自检项在 F1 不适用——循环 DMA 本身持续搬运；
 *  - 只出外设级 API（原始码/均值），物理量换算在 Modules/power（标定点唯一）；
 *  - 缓冲与窗口样本数 = ROBOT_POWER_WINDOW（robot_config.h，编译期定死）。
 */
#ifndef F103RC_BSP_ADC_H
#define F103RC_BSP_ADC_H

#include <stdint.h>
#include <stdbool.h>

/* 双同步原始对：ADC1 联合 DR 一次 32 位搬运，低 16=ADC1、高 16=ADC2
 * （采样对象归属见 bsp_pin.h：IN4=PA4 电流 / IN5=PA5 电压） */
typedef struct
{
    uint16_t raw_i;     /* ADC1 IN4 原始码（0..4095） */
    uint16_t raw_v;     /* ADC2 IN5 原始码（0..4095） */
} AdcPair_t;

/* ADC3 通道序号（对应 bsp_pin.h 的 PIN_ADC3_CH1/CH2，采样对象归属见其注释） */
#define ADC3_CH1     1u
#define ADC3_CH2     2u

/**
 * @brief  启动采样（幂等，重复调用无副作用）。
 *         校准 ADC1/2/3（F1 上电必须各校准一次，且在启动之前）→
 *         双同步 MultiModeStart_DMA 循环搬运 → 丢前 2 窗首批样本
 *         （约 2×(16 对×5.7µs) ≈ 200µs 忙等，之后缓冲全量为新鲜数据）→
 *         ADC3 降为单通道轮询（原因见 Adc3_Read 注释，重跑 HAL_ADC_Init 生效，
 *         CubeMX 的 2-rank 扫描基线在运行期被此覆盖属预期）。
 *         不重跑 MX_ADCx_Init（main.c 已在调度器启动前调用）。
 * @note   需 Bsp_Init/Log_Init 已跑（main.c USER CODE 2 中按序调用）；
 *         ADC3 不在此启动：软件触发由 Adc3_Read 每次自启。
 */
void Adc_Init(void);

/**
 * @brief  取最新一对同步样本（uint32 在 Cortex-M3 上读原子，无撕裂）。
 * @param  out 接收原始对，不可为 NULL
 * @retval false = out 为 NULL / 未初始化（Adc_Init 未跑过）
 */
bool Adc_GetLatest(AdcPair_t *out);

/**
 * @brief  取全窗均值（窗长 = ROBOT_POWER_WINDOW 对）。
 *         逐对累加：32 位对读原子（Cortex-M3），各对之间可能跨 DMA 轮次
 *         新旧混合——对均值影响可忽略（方案 B 指南 §4.2 简化做法，PRIMASK
 *         关中断本就拦不住 DMA 硬件写入，不做伪快照）。
 *         高频噪声由硬件采样窗滤除，负载变化滤除由调用方周期节拍负责。
 * @retval false = out 为 NULL / 未初始化
 */
bool Adc_GetAvg(AdcPair_t *out);

/**
 * @brief  ADC3 单通道读取（独立慢速路，软件触发单次转换）。
 * @param  ch ADC3_CH1 / ADC3_CH2
 * @param  raw 接收原始码，不可为 NULL
 * @retval false = 参数非法 / 超时（ROBOT_ADC_TIMEOUT_MS）/ HAL 错误
 * @note   ⚠ 实现说明：CubeMX 基线是 2-rank 扫描，但 F1 只有一个 DR 且扫描
 *         模式下 EOC 仅在序列末置位（stm32f1xx_hal_adc.c 2346 行注明），
 *         轮询路径只能取到 rank2——因此 Adc_Init 把 ADC3 降为单通道，
 *         本函数每次读前重配 rank1 通道（HAL 官方 API，µs 级）。
 *         阻塞约 68 周期 ≈ 6µs + 轮询余量，20~50ms 级调用方无感。
 */
bool Adc3_Read(uint8_t ch, uint16_t *raw);

/**
 * @brief  诊断：双同步 DMA 采样是否已启动（Adc_Init 跑过且 HAL 报 OK）
 */
bool Adc_IsReady(void);

#endif /* F103RC_BSP_ADC_H */
