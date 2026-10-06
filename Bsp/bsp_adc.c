/**
 * @file    bsp_adc.c
 * @brief   ADC1/2 双同步 DMA 循环采样（双 rank：V/I + J0/J1）实现
 *
 * CubeMX 基线（adc.c，方案 B 的 F4 演进形）：ADC1 主 + ADC2 从双同步 REGSIMULT、
 * 连续转换、双 rank 等长序列——rank1 对 = (IN4 电流, IN5 电压)，rank2 对 =
 * (IN12 J0, IN13 J1)；DMA2_Stream0 循环（Word 对齐），字流 = rank1 对、rank2 对
 * 交替。DMA 中断已在 main.c 屏蔽。
 *
 * ⚠ F1→F4 迁移勘误速查（2026-10-05 定案；全文/证据链见 docs/BSP调试日志.md）：
 *  #1 F4 无 ADC 校准 API（F1 的 Calibration_Start 已删）；
 *  #2 从机 EXTTRIG 运行期补位不再需要（F4 主机触发硬件同步从机）；
 *  #3 DMAContinuousRequests=ENABLE 必须 .ioc 源头保证（驱动 CCR.DDS）；
 *  #4 多模式 DMA 读 CDR：低16=ADC1 / 高16=ADC2，与 F1 DR 打包一致；
 *  #5 F4 HAL 只使能多模式主机——从机 ADON 须 Adc_Init 运行期补位
 *     （否则 OVR 死锁，J-Link 活体实锤）；
 *  #6 MULTI≠0 屏蔽 ADC3 独立启动——ADC3 退役，J0/J1 并入 rank2（本文件）。
 */
#include "bsp_adc.h"
#include "bsp_pin.h"
#include "bsp_sys.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "adc.h"
#include "dma.h"
#include "stm32f4xx_hal.h"

/* CubeMX 在 adc.c 定义、未在任何头文件 extern（DMA 挂在 ADC1 MspInit 里）；
 * BSP 允许 extern CubeMX 句柄（规划 一.2），NDTR 反推最新下标要用它 */
extern DMA_HandleTypeDef hdma_adc1;

/* 双同步样本环形缓冲：DMA 循环覆盖，1 Word = 1 对（低16=ADC1/高16=ADC2）。
 * 字流按 rank 交替：偶下标 = rank1 对(IN4,IN5)=电源 V/I，奇下标 = rank2 对
 * (IN12,IN13)=关节 J0/J1。缓冲总长 = 2×窗口长（V/I 均值窗仍为
 * ROBOT_POWER_WINDOW 对）。
 * 16 对 × 4.6µs ≈ 73µs 窗，覆盖约 219 k对/rank/s 连续采样的高频噪声 */
static uint32_t s_pair_buf[2u * ROBOT_POWER_WINDOW];

static bool s_started = false;

void Adc_Init(void)
{
    if (s_started)
    {
        return;
    }
    /* MX_ADCx_Init 未跑（Instance 空）则拒绝启动——CMSIS 实例宏≠HAL 句柄（S0 教训） */
    if (hadc1.Instance != ADC1 || hadc2.Instance != ADC2)
    {
        Log_Printf("[ADC] MX_ADCx_Init 未跑，拒绝启动\r\n");
        return;
    }

    /* F4 老代 ADC IP 无校准（见文件头勘误 #1/#2），直接启动。
     * ⚠ 勘误 #5：先手动上电从机（F4 HAL 只使能主机），主机 MultiModeStart
     * 内置的 Tstab 稳定等待顺带覆盖从机上电稳定，之后从机才接得住主机的
     * 同步触发 */
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC | ADC_FLAG_OVR);

    /* 双同步专用启动（普通 Start_DMA 在多模式下返回 HAL_ERROR，F1/F4 同此约束）。
     * Length 语义 = DMA 传输项数，Word 宽度下 1 项 = 1 对同步样本；
     * 循环持续搬运的前提 = .ioc 的 DMAContinuousRequests=ENABLE（勘误 #3） */
    if (HAL_ADCEx_MultiModeStart_DMA(&hadc1, s_pair_buf,
                                     (uint32_t)(2u * ROBOT_POWER_WINDOW)) != HAL_OK)
    {
        Log_Printf("[ADC] 双同步 DMA 启动失败\r\n");
        return;
    }

    /* 丢前 2 轮首批样本（BSP 坑 #4）：一轮 = 2×16 对×4.6µs ≈ 147µs，
     * 取 300µs ≈ 2 轮余量，之后缓冲全量为上电以来的新鲜数据（依赖
     * main.c 中 Bsp_Init 已跑） */
    Bsp_DelayUs(300u);

    s_started = true;
}

bool Adc_GetLatest(AdcPair_t *out)
{
    if (out == NULL || !s_started)
    {
        return false;
    }

    /* 由 DMA 剩余计数反推最近写完的下标：写完 buf[i] 后 NDTR = N-1-i；
     * NDTR>=N（刚重载）或读到 0 的瞬间，最近写完的都是 buf[N-1]。
     * 读 NDTR 到读缓冲之间 DMA 可能已覆盖该下标——读到的是更新的一对，
     * 任意时刻都是完整同步对（32 位读原子，无半字撕裂）。
     * V/I 取偶下标（rank1 对）；若最新是奇位则回退一格取上一轮 rank1 */
    uint32_t total = 2u * ROBOT_POWER_WINDOW;
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_adc1);
    if (ndtr >= total || (total - ndtr) < 2u)
    {
        return false;                       /* 首轮 rank1 对未落 */
    }
    uint32_t newest = total - 1u - ndtr;
    uint32_t vidx = (newest % 2u == 0u) ? newest : (newest - 1u);

    uint32_t word = s_pair_buf[vidx];
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
        uint32_t word = s_pair_buf[2u * k]; /* 偶下标 = rank1 对；32 位读原子 */
        sum_i += word & 0xFFFFu;
        sum_v += (word >> 16) & 0xFFFFu;
    }
    out->raw_i = (uint16_t)(sum_i / ROBOT_POWER_WINDOW);
    out->raw_v = (uint16_t)(sum_v / ROBOT_POWER_WINDOW);
    return true;
}

bool Adc_J_Read(uint8_t ch, uint16_t *raw)
{
    if (raw == NULL || !s_started || (ch != ADC_J_CH1 && ch != ADC_J_CH2))
    {
        return false;
    }

    /* 关节对 = 奇下标（rank2）：最新 rank1 落在偶位时，本轮 rank2 尚未落，
     * 取上一轮的（滞后一个转换 ≈ 9.2µs，20~50ms 级调用方无感） */
    uint32_t total = 2u * ROBOT_POWER_WINDOW;
    uint32_t ndtr = __HAL_DMA_GET_COUNTER(&hdma_adc1);
    if (ndtr >= total || (total - ndtr) < 2u)
    {
        return false;                       /* 首轮 J 对未落 */
    }
    uint32_t newest = total - 1u - ndtr;
    uint32_t jidx = (newest % 2u == 1u) ? newest : (newest - 1u);

    uint32_t word = s_pair_buf[jidx];
    *raw = (ch == ADC_J_CH1) ? (uint16_t)(word & 0xFFFFu)            /* 低16=ADC1 IN12 */
                             : (uint16_t)((word >> 16) & 0xFFFFu);   /* 高16=ADC2 IN13 */
    return true;
}

bool Adc_IsReady(void)
{
    return s_started;
}
