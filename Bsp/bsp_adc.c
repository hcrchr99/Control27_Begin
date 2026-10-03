/**
 * @file    bsp_adc.c
 * @brief   ADC1/2 双同步 DMA 循环采样 + ADC3 单通道轮询实现
 *
 * CubeMX 基线（adc.c，方案 B）：ADC1 主 + ADC2 从双同步 REGSIMULT、连续转换、
 * DMA1_Ch1 循环（Word 对齐）；ADC3 独立 55.5 周期采样。DMA 中断已在 main.c 屏蔽。
 *
 * ⚠ 两处 F1 特有勘误（2026-10-03 预研，HAL 源码核实）：
 *  1. 双同步启动必须 HAL_ADCEx_MultiModeStart_DMA——HAL_ADC_Start_DMA 在多模式下
 *     直接返回 HAL_ERROR（stm32f1xx_hal_adc.c 多模式分支）；
 *  2. F103 的 ADC CR2 无 DDS 位，方案 B 指南 §2.1/§3 的"DMA Continuous Requests
 *     = Enabled"自检项是 F4/L4 系概念，F1 无此配置也无需配置，循环 DMA 持续搬运。
 */
#include "bsp_adc.h"
#include "bsp_pin.h"
#include "bsp_sys.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "adc.h"
#include "dma.h"
#include "stm32f1xx_hal.h"

/* CubeMX 在 adc.c 定义、未在任何头文件 extern（DMA 挂在 ADC1 MspInit 里）；
 * BSP 允许 extern CubeMX 句柄（规划 一.2），CNDTR 反推最新下标要用它 */
extern DMA_HandleTypeDef hdma_adc1;

/* 双同步样本环形缓冲：DMA 循环覆盖，1 Word = 1 对（低16=ADC1/高16=ADC2）。
 * 窗长即均值窗口（robot_config.h，"窗口均值样本数进配置"的落实）。
 * 16 对 × 5.7µs ≈ 91µs 窗，覆盖约 176 k对/s 连续采样的高频噪声 */
static uint32_t s_pair_buf[ROBOT_POWER_WINDOW];

static bool s_started = false;

void Adc_Init(void)
{
    if (s_started)
    {
        return;
    }
    /* MX_ADCx_Init 未跑（Instance 空）则拒绝启动——CMSIS 实例宏≠HAL 句柄（S0 教训） */
    if (hadc1.Instance != ADC1 || hadc2.Instance != ADC2 || hadc3.Instance != ADC3)
    {
        Log_Printf("[ADC] MX_ADCx_Init 未跑，拒绝启动\r\n");
        return;
    }

    /* F1 上电必须校准（内部自带停机→使能→等 2 ADC 周期时序），且在启动之前 */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc3) != HAL_OK)
    {
        Log_Printf("[ADC] 校准失败\r\n");
        return;
    }

    /* ⚠ F1 双同步从机两个缺失位（板上实测破案 2026-10-06）：
     * 1) CR2.EXTTRIG 无人置位——HAL_ADC_Init 有意不置（注释明说留给 Start_xxx），
     *    而 HAL_ADCEx_MultiModeStart_DMA 只给主机置 EXTTRIG|SWSTART，其头部注释
     *    "slave trigger already done into HAL_ADC_Init" 与实现自相矛盾；不补则
     *    从机永不触发，DR 高半字冻结在启动残留值（实测恒 2000）。
     * 2) CR2.CONT 从机为 DISABLE——CubeMX 对从机生成的 ContinuousConvMode=DISABLE，
     *    主机 SWSTART 边沿只同步触发从机一拍（采到启动瞬间悬空 PA5≈轨电平 4091），
     *    此后主机连续自触发不再产生新边沿，从机停转（实测 PA5 接 GND 读数不动）。
     *    从机必须与主机同为连续模式。补 CONT 需重跑 HAL_ADC_Init（F1 允许使能
     *    空闲态重配，与 ADC3 降单通道同一手法）；注意 Init 会清 EXTTRIG，故
     *    SET_BIT 必须在重跑之后 */
    hadc2.Init.ContinuousConvMode = ENABLE;
    if (HAL_ADC_Init(&hadc2) != HAL_OK)
    {
        Log_Printf("[ADC] ADC2 连续模式重配失败\r\n");
        return;
    }
    SET_BIT(hadc2.Instance->CR2, ADC_CR2_EXTTRIG);

    /* ⚠ 双同步专用启动（普通 Start_DMA 在多模式下返回 HAL_ERROR）。
     * Length 语义 = DMA 传输项数，Word 宽度下 1 项 = 1 对同步样本 */
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, s_pair_buf, ROBOT_POWER_WINDOW) != HAL_OK)
    {
        Log_Printf("[ADC] 双同步 DMA 启动失败\r\n");
        return;
    }

    /* ADC3 降为单通道：F1 单 DR + 扫描模式 EOC 仅在序列末置位，轮询取不到
     * rank1（详见 bsp_adc.h Adc3_Read 注释）。校准后 ADC 处于使能空闲态，
     * F1 的 HAL_ADC_Init 接受该状态重配 */
    hadc3.Init.NbrOfConversion = 1u;
    if (HAL_ADC_Init(&hadc3) != HAL_OK)
    {
        Log_Printf("[ADC] ADC3 单通道重配失败\r\n");
        return;
    }

    /* 丢前 2 窗首批样本（BSP 坑 #4）：约 2×(16 对×5.7µs)≈114µs，取 200µs 余量，
     * 之后缓冲全量为上电以来的新鲜数据（依赖 main.c 中 Bsp_Init 已跑） */
    Bsp_DelayUs(200u);

    s_started = true;
}

bool Adc_GetLatest(AdcPair_t *out)
{
    if (out == NULL || !s_started)
    {
        return false;
    }

    /* 由 DMA 剩余计数反推最近写完的下标：写完 buf[i] 后 CNDTR = N-1-i；
     * CNDTR>=N（刚重载）或读到 0 的瞬间，最近写完的都是 buf[N-1]。
     * 读 CNDTR 到读缓冲之间 DMA 可能已覆盖该下标——读到的是更新的一对，
     * 任意时刻都是完整同步对（32 位读原子，无半字撕裂） */
    uint32_t cndtr = __HAL_DMA_GET_COUNTER(&hdma_adc1);
    uint32_t idx = (cndtr >= ROBOT_POWER_WINDOW) ? (ROBOT_POWER_WINDOW - 1u)
                                                 : (ROBOT_POWER_WINDOW - 1u - cndtr);

    uint32_t word = s_pair_buf[idx];
    out->raw_i = (uint16_t)(word & 0xFFFFu);            /* 低 16 = ADC1 IN4 */
    out->raw_v = (uint16_t)((word >> 16) & 0xFFFFu);    /* 高 16 = ADC2 IN5 */
    return true;
}

bool Adc_GetAvg(AdcPair_t *out)
{
    if (out == NULL || !s_started)
    {
        return false;
    }

    uint32_t sum_i = 0u;
    uint32_t sum_v = 0u;
    for (uint32_t k = 0u; k < ROBOT_POWER_WINDOW; k++)
    {
        uint32_t word = s_pair_buf[k];      /* 32 位对读原子；跨轮次新旧混合可忽略 */
        sum_i += word & 0xFFFFu;
        sum_v += (word >> 16) & 0xFFFFu;
    }
    out->raw_i = (uint16_t)(sum_i / ROBOT_POWER_WINDOW);
    out->raw_v = (uint16_t)(sum_v / ROBOT_POWER_WINDOW);
    return true;
}

bool Adc3_Read(uint8_t ch, uint16_t *raw)
{
    if (raw == NULL || !s_started || (ch != ADC3_CH1 && ch != ADC3_CH2))
    {
        return false;
    }

    /* 每次读前重配 rank1 通道（HAL 官方 API，µs 级；通道宏来自 bsp_pin.h 唯一硬件地图） */
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = (ch == ADC3_CH1) ? PIN_ADC3_CH1 : PIN_ADC3_CH2;
    cfg.Rank         = ADC_REGULAR_RANK_1;
    cfg.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;   /* 与 CubeMX 基线一致，源阻抗容限 */
    if (HAL_ADC_ConfigChannel(&hadc3, &cfg) != HAL_OK)
    {
        return false;
    }
    if (HAL_ADC_Start(&hadc3) != HAL_OK)
    {
        return false;
    }

    bool ok = (HAL_ADC_PollForConversion(&hadc3, ROBOT_ADC_TIMEOUT_MS) == HAL_OK);
    if (ok)
    {
        *raw = (uint16_t)HAL_ADC_GetValue(&hadc3);
    }
    /* 复位状态机（超时路径也走此清理，保证下次 ConfigChannel 前无转换进行） */
    (void)HAL_ADC_Stop(&hadc3);
    return ok;
}

bool Adc_IsReady(void)
{
    return s_started;
}
