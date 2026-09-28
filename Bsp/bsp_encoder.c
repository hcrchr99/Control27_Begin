/**
 * @file    bsp_encoder.c
 * @brief   四路正交编码器实现：静态句柄表 + 16bit 回绕差分
 */
#include "bsp_encoder.h"
#include "bsp_pin.h"
#include "tim.h"

/* HAL 句柄表（extern 自 tim.h）。
 * ⚠ S0 教训：TIM1/TIM2/... 实例宏是寄存器块指针（USART_TypeDef* 同理），
 * 严禁当句柄用；这里一律 &htimx。 */
static TIM_HandleTypeDef *const s_enc_tim[4] = { &htim1, &htim2, &htim3, &htim8 };

/* 各通道上次计数快照，仅 Encoder_Read（单一读者）访问 */
static uint16_t s_enc_last[4] = { 0u, 0u, 0u, 0u };

/* GetCount 的独立快照与累计：与 Encoder_Read 互不干扰（各消费各的 CNT 差分） */
static uint16_t s_pos_last[4] = { 0u, 0u, 0u, 0u };
static int32_t  s_pos_acc[4]  = { 0, 0, 0, 0 };

void Encoder_InitAll(void)
{
    for (uint32_t i = 0; i < 4u; i++)
    {
        /* 编码器模式两通道都要 Start 计数才走起来 */
        (void)HAL_TIM_Encoder_Start(s_enc_tim[i], TIM_CHANNEL_1);
        (void)HAL_TIM_Encoder_Start(s_enc_tim[i], TIM_CHANNEL_2);
    }
}

int32_t Encoder_Read(uint8_t ch)
{
    if (ch < ENC_CH1 || ch > ENC_CH4)
    {
        return 0;
    }
    uint32_t idx = ch - ENC_CH1;

    uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(s_enc_tim[idx]);
    uint16_t last = s_enc_last[idx];
    s_enc_last[idx] = now;

    /* uint16 差值经 int16 解释，计数回绕（0→65535 / 65535→0）天然正确 */
    int16_t diff = (int16_t)(now - last);
    return (int32_t)diff;
}

int32_t Encoder_GetCount(uint8_t ch)
{
    if (ch < ENC_CH1 || ch > ENC_CH4)
    {
        return 0;
    }
    uint32_t idx = ch - ENC_CH1;

    uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(s_enc_tim[idx]);
    int16_t diff = (int16_t)(now - s_pos_last[idx]);
    s_pos_last[idx] = now;
    s_pos_acc[idx] += diff;     /* int32 累计，±21.4 亿计数内不回绕 */
    return s_pos_acc[idx];
}
