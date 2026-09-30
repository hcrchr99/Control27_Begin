/**
 * @file    bsp_pin.h
 * @brief   F103C8_PTX_T 板级引脚地图（测试发送板专用）
 *
 * 与车端 Bsp/bsp_pin.h 同约定：本文件是板级唯一硬件地图，宏名保持与车端
 * 相同的语义组（PIN_WL_* / GpioPin_t），使 Modules/remote 零改动移植；
 * 具体引脚按本板 CubeMX 标签（WL_CSN=PB1 / WL_CE=PB2 / WL_IRQ=PB10）。
 * 车端的 DIR/PWM/ENC/ADC 组本板不存在，不定义。
 */
#ifndef F103C8_PTX_BSP_PIN_H
#define F103C8_PTX_BSP_PIN_H

#include "main.h"

/* 引脚标识对象：与车端同型，值传递 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} GpioPin_t;

/* 无线模块控制脚（CubeMX 标签 WL_CSN/WL_CE/WL_IRQ）
 * CSN 空闲必须为高（由 Spi_Init 强制，建议 CubeMX 默认电平也设 High）；
 * IRQ=PB10 下降沿 EXTI + 上拉，EXTI15_10 NVIC 优先级 5 */
#define PIN_WL_CSN_GPIO_PORT    WL_CSN_GPIO_Port
#define PIN_WL_CSN_GPIO_PIN     WL_CSN_Pin
#define PIN_WL_CSN              ((GpioPin_t){ PIN_WL_CSN_GPIO_PORT, PIN_WL_CSN_GPIO_PIN })
#define PIN_WL_CE_GPIO_PORT     WL_CE_GPIO_Port
#define PIN_WL_CE_GPIO_PIN      WL_CE_Pin
#define PIN_WL_CE               ((GpioPin_t){ PIN_WL_CE_GPIO_PORT, PIN_WL_CE_GPIO_PIN })
#define PIN_WL_IRQ_GPIO_PORT    WL_IRQ_GPIO_Port
#define PIN_WL_IRQ_GPIO_PIN     WL_IRQ_Pin
#define PIN_WL_IRQ_EXTI_IRQn    WL_IRQ_EXTI_IRQn
#define PIN_WL_IRQ              ((GpioPin_t){ PIN_WL_IRQ_GPIO_PORT, PIN_WL_IRQ_GPIO_PIN })

/* ⚠ CMSIS 实例宏（SPI1/USART1）是寄存器块指针，HAL 句柄一律用
 * hspi1/huart1（各外设头文件里 extern）——车端 S0 教训，同理适用本板。 */

#endif /* F103C8_PTX_BSP_PIN_H */
