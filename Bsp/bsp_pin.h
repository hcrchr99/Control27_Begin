/**
 * @file    bsp_pin.h
 * @brief   BSP 层唯一硬件地图：引脚 / 外设实例 / 通道号宏
 *
 * 规则（BSP层开发规划 一.5）：
 *  - 与 CubeMX 已生成的宏（main.h）保持一致，本文件只做语义别名，不重复定义，
 *    避免双源真相；main.h 没有的（方向 GPIO、外设通道映射等）在此补齐。
 *  - 引脚变更：CubeMX 改配置后同步本文件，两处必须一致。
 *  - 宏名只用外设概念，禁止器件名（Motor/Servo/Oled/Nrf24）；
 *    器件归属写在注释里，供 Modules 层查阅。
 */
#ifndef F103RC_BSP_PIN_H
#define F103RC_BSP_PIN_H

#include "main.h"

/* 引脚标识对象：BSP/Modules 接口统一用它传脚，值传递，构造用下方 PIN_xxx 宏 */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
} GpioPin_t;

/* ================================ 数字输出 ================================ */

/* LED1（PB12） */
#define PIN_LED1_GPIO_PORT      LED1_GPIO_Port
#define PIN_LED1_GPIO_PIN       LED1_Pin
#define PIN_LED1                ((GpioPin_t){ PIN_LED1_GPIO_PORT, PIN_LED1_GPIO_PIN })

/* LED2（PC13，驱动能力仅 ~3mA，限流电阻 >=1k，低电平点亮，禁止挪作他用） */
#define PIN_LED2_GPIO_PORT      LED2_GPIO_Port
#define PIN_LED2_GPIO_PIN       LED2_Pin
#define PIN_LED2                ((GpioPin_t){ PIN_LED2_GPIO_PORT, PIN_LED2_GPIO_PIN })

/* 蜂鸣器（PC4） */
#define PIN_ALARM_GPIO_PORT     Alarm_GPIO_Port
#define PIN_ALARM_GPIO_PIN      Alarm_Pin
#define PIN_ALARM               ((GpioPin_t){ PIN_ALARM_GPIO_PORT, PIN_ALARM_GPIO_PIN })

/* 功率级待机控制（PA10，TB6612 STBY；CubeMX 复位电平 = 低 = 关断） */
#define PIN_PWR_STBY_GPIO_PORT  STBY_GPIO_Port
#define PIN_PWR_STBY_GPIO_PIN   STBY_Pin
#define PIN_PWR_STBY            ((GpioPin_t){ PIN_PWR_STBY_GPIO_PORT, PIN_PWR_STBY_GPIO_PIN })

/* 方向 GPIO ×8（PB0/PB1/PA6/PA7/PC0/PC1/PC8/PC9）
 * 每 2 个一组：xA=xIN1, xB=xIN2；组序号与电机序号的对应关系
 * 由硬件按布线最终决定（README 备注序号未定，勿在 BSP 固化映射）。
 * 2026-09-28 变更：原 PB4/PB5 组让位给编码器3（TIM3 部分重映射），
 * 方向组迁至 PA6/PA7。 */
#define PIN_DIR1A_GPIO_PORT     GPIOB
#define PIN_DIR1A_GPIO_PIN      GPIO_PIN_0
#define PIN_DIR1B_GPIO_PORT     GPIOB
#define PIN_DIR1B_GPIO_PIN      GPIO_PIN_1
#define PIN_DIR2A_GPIO_PORT     GPIOA
#define PIN_DIR2A_GPIO_PIN      GPIO_PIN_6
#define PIN_DIR2B_GPIO_PORT     GPIOA
#define PIN_DIR2B_GPIO_PIN      GPIO_PIN_7
#define PIN_DIR3A_GPIO_PORT     GPIOC
#define PIN_DIR3A_GPIO_PIN      GPIO_PIN_0
#define PIN_DIR3B_GPIO_PORT     GPIOC
#define PIN_DIR3B_GPIO_PIN      GPIO_PIN_1
#define PIN_DIR4A_GPIO_PORT     GPIOC
#define PIN_DIR4A_GPIO_PIN      GPIO_PIN_8
#define PIN_DIR4B_GPIO_PORT     GPIOC
#define PIN_DIR4B_GPIO_PIN      GPIO_PIN_9

#define PIN_DIR1A               ((GpioPin_t){ PIN_DIR1A_GPIO_PORT, PIN_DIR1A_GPIO_PIN })
#define PIN_DIR1B               ((GpioPin_t){ PIN_DIR1B_GPIO_PORT, PIN_DIR1B_GPIO_PIN })
#define PIN_DIR2A               ((GpioPin_t){ PIN_DIR2A_GPIO_PORT, PIN_DIR2A_GPIO_PIN })
#define PIN_DIR2B               ((GpioPin_t){ PIN_DIR2B_GPIO_PORT, PIN_DIR2B_GPIO_PIN })
#define PIN_DIR3A               ((GpioPin_t){ PIN_DIR3A_GPIO_PORT, PIN_DIR3A_GPIO_PIN })
#define PIN_DIR3B               ((GpioPin_t){ PIN_DIR3B_GPIO_PORT, PIN_DIR3B_GPIO_PIN })
#define PIN_DIR4A               ((GpioPin_t){ PIN_DIR4A_GPIO_PORT, PIN_DIR4A_GPIO_PIN })
#define PIN_DIR4B               ((GpioPin_t){ PIN_DIR4B_GPIO_PORT, PIN_DIR4B_GPIO_PIN })

/* ================================ 数字输入 ================================ */

/* 按键 1/2（PA11/PA12，下降沿 EXTI，内部上拉） */
#define PIN_KEY1_GPIO_PORT      KEY1_GPIO_Port
#define PIN_KEY1_GPIO_PIN       KEY1_Pin
#define PIN_KEY1_EXTI_IRQn      KEY1_EXTI_IRQn
#define PIN_KEY1                ((GpioPin_t){ PIN_KEY1_GPIO_PORT, PIN_KEY1_GPIO_PIN })

#define PIN_KEY2_GPIO_PORT      KEY2_GPIO_Port
#define PIN_KEY2_GPIO_PIN       KEY2_Pin
#define PIN_KEY2_EXTI_IRQn      KEY2_EXTI_IRQn
#define PIN_KEY2                ((GpioPin_t){ PIN_KEY2_GPIO_PORT, PIN_KEY2_GPIO_PIN })

/* 无线模块控制脚（CSN=PC12 空闲必须为高；CE=PD2；IRQ=PC5 下降沿 EXTI） */
#define PIN_WL_CSN_GPIO_PORT    nRF24_CSN_GPIO_Port
#define PIN_WL_CSN_GPIO_PIN     nRF24_CSN_Pin
#define PIN_WL_CSN              ((GpioPin_t){ PIN_WL_CSN_GPIO_PORT, PIN_WL_CSN_GPIO_PIN })
#define PIN_WL_CE_GPIO_PORT     nRF24_CE_GPIO_Port
#define PIN_WL_CE_GPIO_PIN      nRF24_CE_Pin
#define PIN_WL_CE               ((GpioPin_t){ PIN_WL_CE_GPIO_PORT, PIN_WL_CE_GPIO_PIN })
#define PIN_WL_IRQ_GPIO_PORT    nRF24_IRQ_GPIO_Port
#define PIN_WL_IRQ_GPIO_PIN     nRF24_IRQ_Pin
#define PIN_WL_IRQ_EXTI_IRQn    nRF24_IRQ_EXTI_IRQn
#define PIN_WL_IRQ              ((GpioPin_t){ PIN_WL_IRQ_GPIO_PORT, PIN_WL_IRQ_GPIO_PIN })

/* ================================= PWM 输出 ================================ */

/* 20kHz PWM 组（TIM4 CH1-4，PSC=0 ARR=3599；用途：4 路电机调速） */
#define PIN_PWM20K_TIM          TIM4
#define PIN_PWM20K_CH1          TIM_CHANNEL_1   /* PB6 */
#define PIN_PWM20K_CH2          TIM_CHANNEL_2   /* PB7 */
#define PIN_PWM20K_CH3          TIM_CHANNEL_3   /* PB8 */
#define PIN_PWM20K_CH4          TIM_CHANNEL_4   /* PB9 */
#define PIN_PWM20K_ARR          3599u

/* 50Hz PWM 组（TIM5 CH1-4，PSC=1439 ARR=999，1 step = 20us；
 * 用途：舵机脉宽控制，CH4 为备用通道） */
#define PIN_PWM50HZ_TIM         TIM5
#define PIN_PWM50HZ_CH1         TIM_CHANNEL_1   /* PA0 */
#define PIN_PWM50HZ_CH2         TIM_CHANNEL_2   /* PA1 */
#define PIN_PWM50HZ_CH3         TIM_CHANNEL_3   /* PA2 */
#define PIN_PWM50HZ_CH4         TIM_CHANNEL_4   /* PA3 */
#define PIN_PWM50HZ_ARR         999u

/* ================================ 编码器输入 =============================== */

/* TI12 四倍频，IC Filter = 5；BSP 只出原始增量，换算系数在 robot_config.h
 * ⚠ ENC2/ENC3 均走重映射，其 MspInit 的 remap 宏会改写 SWJ_CFG，
 * 两处 USER CODE 段的 __HAL_AFIO_REMAP_SWJ_NOJTAG() 防御必须保留。 */
#define PIN_ENC1_TIM            TIM1    /* PA8/PA9  */
#define PIN_ENC2_TIM            TIM2    /* PA15/PB3（部分重映射1，已禁 JTAG） */
#define PIN_ENC3_TIM            TIM3    /* PB4/PB5（部分重映射，2026-09-28 由 PA6/PA7 迁入） */
#define PIN_ENC4_TIM            TIM8    /* PC6/PC7  */

/* 编码器输入引脚对象宏（Gpio_Read 读原始电平，诊断信号通断用；
 * 注意 ENC3 实际引脚是重映射后的 PB4/PB5） */
#define PIN_ENC1A_GPIO_PORT     GPIOA
#define PIN_ENC1A_GPIO_PIN      GPIO_PIN_8
#define PIN_ENC1B_GPIO_PORT     GPIOA
#define PIN_ENC1B_GPIO_PIN      GPIO_PIN_9
#define PIN_ENC2A_GPIO_PORT     GPIOA
#define PIN_ENC2A_GPIO_PIN      GPIO_PIN_15
#define PIN_ENC2B_GPIO_PORT     GPIOB
#define PIN_ENC2B_GPIO_PIN      GPIO_PIN_3
#define PIN_ENC3A_GPIO_PORT     GPIOB
#define PIN_ENC3A_GPIO_PIN      GPIO_PIN_4
#define PIN_ENC3B_GPIO_PORT     GPIOB
#define PIN_ENC3B_GPIO_PIN      GPIO_PIN_5
#define PIN_ENC4A_GPIO_PORT     GPIOC
#define PIN_ENC4A_GPIO_PIN      GPIO_PIN_6
#define PIN_ENC4B_GPIO_PORT     GPIOC
#define PIN_ENC4B_GPIO_PIN      GPIO_PIN_7

#define PIN_ENC1A               ((GpioPin_t){ PIN_ENC1A_GPIO_PORT, PIN_ENC1A_GPIO_PIN })
#define PIN_ENC1B               ((GpioPin_t){ PIN_ENC1B_GPIO_PORT, PIN_ENC1B_GPIO_PIN })
#define PIN_ENC2A               ((GpioPin_t){ PIN_ENC2A_GPIO_PORT, PIN_ENC2A_GPIO_PIN })
#define PIN_ENC2B               ((GpioPin_t){ PIN_ENC2B_GPIO_PORT, PIN_ENC2B_GPIO_PIN })
#define PIN_ENC3A               ((GpioPin_t){ PIN_ENC3A_GPIO_PORT, PIN_ENC3A_GPIO_PIN })
#define PIN_ENC3B               ((GpioPin_t){ PIN_ENC3B_GPIO_PORT, PIN_ENC3B_GPIO_PIN })
#define PIN_ENC4A               ((GpioPin_t){ PIN_ENC4A_GPIO_PORT, PIN_ENC4A_GPIO_PIN })
#define PIN_ENC4B               ((GpioPin_t){ PIN_ENC4B_GPIO_PORT, PIN_ENC4B_GPIO_PIN })

/* ================================ ADC 采样 ================================ */

/* ADC1+ADC2 双同步规则组 + 连续转换 + DMA1_Ch1 循环（32位打包：低16=ADC1，高16=ADC2） */
#define PIN_ADC_BATTERY_ADC     ADC1            /* PA4 电流 */
#define PIN_ADC_BATTERY_CH      ADC_CHANNEL_4
#define PIN_ADC2_SYNC_CH        ADC_CHANNEL_5   /* PA5 电压（ADC2 同步采样） */

/* ADC3 独立扫描轮询（关节电流 ×2） */
#define PIN_ADC3_TIM            ADC3
#define PIN_ADC3_CH1            ADC_CHANNEL_12  /* PC2 */
#define PIN_ADC3_CH2            ADC_CHANNEL_13  /* PC3 */

/* ================================== 串口 ================================== */

/* 调试日志口 UART4（PC10/PC11，115200，纯中断收发，无 DMA）。
 * ⚠ 注意：CubeMX 句柄是 huart4（extern 自 usart.h）。
 * CMSIS 的 `UART4` 宏是外设寄存器块指针（USART_TypeDef*），与句柄是两回事，
 * 严禁强转后当句柄传给 HAL——同理 TIM4/ADC1/SPI2 等实例宏都只是寄存器块，
 * HAL 句柄一律用 htim4/hadc1/hspi2（各外设头文件里 extern）。 */

#endif /* F103RC_BSP_PIN_H */
