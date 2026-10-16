/**
 * @file    bsp_adc.c
 * @brief   摇杆双通道 ADC 读取实现（单 rank 逐次读，见 bsp_adc.h）
 */
#include "bsp_adc.h"
#include "robot_config.h"
#include "main.h"
#include "stm32f1xx_hal.h"

extern ADC_HandleTypeDef hadc1;

/* 单通道单转换：重配 rank 通道 → 启动 → 轮询 EOC → 取值 */
static bool Adc_ReadChan(uint32_t channel, uint16_t *out)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        return false;
    }
    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return false;
    }
    if (HAL_ADC_PollForConversion(&hadc1, ROBOT_ADC_TIMEOUT_MS) != HAL_OK)
    {
        (void)HAL_ADC_Stop(&hadc1);
        return false;
    }
    if (out != NULL)
    {
        *out = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    return true;
}

bool Adc_IsReady(void)
{
    return (hadc1.Instance == ADC1) && (hadc1.State == HAL_ADC_STATE_READY);
}

bool Adc_JoyRead(uint16_t *x_raw, uint16_t *y_raw)
{
    if ((!Adc_IsReady()) || (x_raw == NULL && y_raw == NULL))
    {
        return false;
    }
    /* 先读 Y 后读 X：任一通道失败即整对失败，失败侧输出保持原值 */
    if (!Adc_ReadChan(ADC_CHANNEL_2, y_raw))
    {
        return false;
    }
    return Adc_ReadChan(ADC_CHANNEL_1, x_raw);
}
