/**
 * @file    bsp_pin.h
 * @brief   Remoter 遥控器板级引脚地图（原理图 ProPrj_遥控_2026-10-13，STM32F103C8T6）
 *
 * 与车端/PTX 测试板 Bsp/bsp_pin.h 同约定：本文件是板级唯一硬件地图，
 * 宏名沿用 GpioPin_t 与 PIN_* 语义组，使 Modules/remote 零改动移植。
 * 引脚来源 = 原理图网表（UserLabel 由 .ioc 的 CubeMX 标签落地到 main.h）：
 *   nRF24L01P（SPI2，APB1 36MHz/8 = 4.5MHz，nRF24 上限 10MHz 内）：
 *     CSN=PB9 / CE=PB8 / IRQ=PB12（EXTI15_10 下降沿 + 上拉，优先级 5）
 *   六只船型开关（双刀单掷，按下=两脚同接地；A/B 是同一开关的两刀，
 *   固件只读 A 脚，B 脚留作备份。全部上拉输入、按下读 0）：
 *     K_1=PA5(A)/PA6(B) 大臂  → 舵机 ID1
 *     K_2=PA7(A)/PB10(B) 小臂 → 舵机 ID2
 *     K_3=PB0(A)/PB3(B) 爪子  → 舵机 ID4（爪开合）
 *     K_4=PA11(A)/PA12(B)     → 爪旋转（舵机 ID5，原理图此组无标题，待上手核对）
 *     K_5=PA3(A)/PA4(B) 手腕  → 舵机 ID3（手腕）
 *     K_EB=PB5（轻触键 SW1，按下=急停）
 *   ⚠ 注意：按键的"物理分组"与"舵机 ID 序号"不一致（K_3 在爪子=ID4、
 *   K_5 在手腕=ID3）。固件按键名→帧字段的映射表在 robot_config.h，
 *   上手若发现和手感不符，改映射表即可，零硬件改动。
 *   摇杆（H2 五针，PS2 摇杆模块）：X=PA1(ADC1_IN1) / Y=PA2(ADC1_IN2) /
 *     SW=PA0（按压键，上拉输入）
 *   LED2 信号灯=PB7（高电平点亮；LED1 电源灯直接挂 3.3V 不占引脚）
 *   日志口=USART1 PA9(TX)/PA10(RX) @115200
 *   HSE 8MHz（X1），SYSCLK=72MHz
 */
#ifndef REMOTER_BSP_PIN_H
#define REMOTER_BSP_PIN_H

#include "main.h"

/* 引脚标识对象：与车端同型，值传递 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} GpioPin_t;

/* 无线模块控制脚——宏名沿用全仓共享约定 PIN_WL_*（WL=wireless，车端/PTX 同名，
 * Modules/remote 零改动移植的前提）；本板 CubeMX 标签是 NRF_*，在此映射：
 * CSN=PB9 / CE=PB8 / IRQ=PB12。CSN 空闲必须为高（由 Spi_Init 强制）；
 * IRQ=PB12 下降沿 EXTI + 上拉，EXTI15_10 NVIC 优先级 5（main.c USER CODE 2 使能） */
#define PIN_WL_CSN_GPIO_PORT    NRF_CSN_GPIO_Port
#define PIN_WL_CSN_GPIO_PIN     NRF_CSN_Pin
#define PIN_WL_CSN              ((GpioPin_t){ PIN_WL_CSN_GPIO_PORT, PIN_WL_CSN_GPIO_PIN })
#define PIN_WL_CE_GPIO_PORT     NRF_CE_GPIO_Port
#define PIN_WL_CE_GPIO_PIN      NRF_CE_Pin
#define PIN_WL_CE               ((GpioPin_t){ PIN_WL_CE_GPIO_PORT, PIN_WL_CE_GPIO_PIN })
#define PIN_WL_IRQ_GPIO_PORT    NRF_IRQ_GPIO_Port
#define PIN_WL_IRQ_GPIO_PIN     NRF_IRQ_Pin
#define PIN_WL_IRQ_EXTI_IRQn    EXTI15_10_IRQn
#define PIN_WL_IRQ              ((GpioPin_t){ PIN_WL_IRQ_GPIO_PORT, PIN_WL_IRQ_GPIO_PIN })

/* 自复位船型开关（中位双断：拨上=A 刀通、拨下=B 刀通、松手全断；按下读 0）。
 * A/B = 同一开关的两个方向刀，⚠ 电气假设 A/B 互斥待上手实测（见 rc_input.h） */
#define PIN_KEY_1A              ((GpioPin_t){ K_1A_GPIO_Port, K_1A_Pin })   /* 大臂+ */
#define PIN_KEY_1B              ((GpioPin_t){ K_1B_GPIO_Port, K_1B_Pin })   /* 大臂- */
#define PIN_KEY_2A              ((GpioPin_t){ K_2A_GPIO_Port, K_2A_Pin })   /* 小臂+ */
#define PIN_KEY_2B              ((GpioPin_t){ K_2B_GPIO_Port, K_2B_Pin })   /* 小臂- */
#define PIN_KEY_3A              ((GpioPin_t){ K_3A_GPIO_Port, K_3A_Pin })   /* 爪子+ */
#define PIN_KEY_3B              ((GpioPin_t){ K_3B_GPIO_Port, K_3B_Pin })   /* 爪子- */
#define PIN_KEY_4A              ((GpioPin_t){ K_4A_GPIO_Port, K_4A_Pin })   /* 爪旋转+(待核对) */
#define PIN_KEY_4B              ((GpioPin_t){ K_4B_GPIO_Port, K_4B_Pin })   /* 爪旋转-(待核对) */
#define PIN_KEY_5A              ((GpioPin_t){ K_5A_GPIO_Port, K_5A_Pin })   /* 手腕+ */
#define PIN_KEY_5B              ((GpioPin_t){ K_5B_GPIO_Port, K_5B_Pin })   /* 手腕- */
#define PIN_KEY_6A              ((GpioPin_t){ K_6A_GPIO_Port, K_6A_Pin })   /* 旋转+ */
#define PIN_KEY_6B              ((GpioPin_t){ K_6B_GPIO_Port, K_6B_Pin })   /* 旋转- */
#define PIN_KEY_EB              ((GpioPin_t){ K_EB_GPIO_Port, K_EB_Pin })   /* 急停(轻触) */
#define PIN_KEY_SW              ((GpioPin_t){ K_SW_GPIO_Port, K_SW_Pin })   /* 摇杆按压(预留) */

/* 信号灯 */
#define PIN_LED1_GPIO_PORT      LED1_GPIO_Port
#define PIN_LED1_GPIO_PIN       LED1_Pin
#define PIN_LED1                ((GpioPin_t){ PIN_LED1_GPIO_PORT, PIN_LED1_GPIO_PIN })

/* ⚠ CMSIS 实例宏（SPI2/USART1/ADC1）是寄存器块指针，HAL 句柄一律用
 * hspi2/huart1/hadc1（main.c 定义，各 Bsp .c 内自行 extern）——车端 S0 教训。 */

#endif /* REMOTER_BSP_PIN_H */
