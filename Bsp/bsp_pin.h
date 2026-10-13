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
#ifndef BSP_PIN_H
#define BSP_PIN_H

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

/* 蜂鸣器：无源件（2026-10-10 起 TIM14 CH1/PA7 方波驱动，见下方 PWM 组的
 * PIN_PWM14_*；响=50% duty / 停=0，音调频率由 TIM14 PSC/ARR 的 .ioc 时基定）。
 * 原方案 PC4 GPIO 高电平直驱仅适用于有源件，PIN_ALARM 宏随 CubeMX 重配删除 */

/* 功率级待机控制（PA10，TB6612 STBY；CubeMX 复位电平 = 低 = 关断） */
#define PIN_PWR_STBY_GPIO_PORT  STBY_GPIO_Port
#define PIN_PWR_STBY_GPIO_PIN   STBY_Pin
#define PIN_PWR_STBY            ((GpioPin_t){ PIN_PWR_STBY_GPIO_PORT, PIN_PWR_STBY_GPIO_PIN })

/* 方向 GPIO ×8（PB0/PB1/PC4/PC5/PC0/PC1/PD3/PD4）
 * 每 2 个一组：xA=xIN1, xB=xIN2；组序号与电机序号的对应关系
 * 由硬件按布线最终决定（README 备注序号未定，勿在 BSP 固化映射）。
 * 2026-09-28 变更：原 PB4/PB5 组让位给编码器3，方向组迁至 PA6/PA7；
 * 2026-10-05 随 F407VG 迁移引脚等位保留；
 * 2026-10-10 变更：DIR2 组 PA6/PA7 → PC4/PC5（PA7 让位 TIM14 CH1 蜂鸣器方波，
 * CubeMX 侧该两脚为无标签 GPIO_Output，宏在此直接写端口）。 */
#define PIN_DIR1A_GPIO_PORT     GPIOE
#define PIN_DIR1A_GPIO_PIN      GPIO_PIN_0
#define PIN_DIR1B_GPIO_PORT     GPIOE
#define PIN_DIR1B_GPIO_PIN      GPIO_PIN_1
#define PIN_DIR2A_GPIO_PORT     GPIOE
#define PIN_DIR2A_GPIO_PIN      GPIO_PIN_3
#define PIN_DIR2B_GPIO_PORT     GPIOE
#define PIN_DIR2B_GPIO_PIN      GPIO_PIN_4
#define PIN_DIR3A_GPIO_PORT     GPIOC
#define PIN_DIR3A_GPIO_PIN      GPIO_PIN_0
#define PIN_DIR3B_GPIO_PORT     GPIOC
#define PIN_DIR3B_GPIO_PIN      GPIO_PIN_1
#define PIN_DIR4A_GPIO_PORT     GPIOD
#define PIN_DIR4A_GPIO_PIN      GPIO_PIN_3
#define PIN_DIR4B_GPIO_PORT     GPIOD
#define PIN_DIR4B_GPIO_PIN      GPIO_PIN_4

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

/* 无线模块控制脚（CSN=PD8 空闲必须为高；CE=PD9；IRQ=PD10 下降沿 EXTI）。
 * 2026-10-07 随 F407VE 换板由 CSN=PC12/CE=PD2/IRQ=PC5 迁入，
 * IRQ 中断线相应由 EXTI9_5 改挂 EXTI15_10（与 KEY1/KEY2 同线） */
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

/* 20kHz PWM 组（TIM4 CH1-4，APB1 定时器 84MHz，PSC=0 ARR=4199；用途：4 路电机调速）
 * ⚠ ARR 为 CubeMX 对账值：改 .ioc 时基必须同步本文件（S5 教训：
 * 不同步则 SetPulseUs/SetDuty 换算出数量级偏差） */
#define PIN_PWM20K_TIM          TIM4
#define PIN_PWM20K_CH1          TIM_CHANNEL_1   /* PB6 */
#define PIN_PWM20K_CH2          TIM_CHANNEL_2   /* PB7 */
#define PIN_PWM20K_CH3          TIM_CHANNEL_3   /* PB8 */
#define PIN_PWM20K_CH4          TIM_CHANNEL_4   /* PB9 */
#define PIN_PWM20K_ARR          4199u

/* 250Hz PWM 组（TIM9 CH1-2，APB2 定时器 168MHz，PSC=335 ARR=1999；
 * 用途：PM10S 数字舵机 250Hz 帧率驱动，1 step = 2µs，帧长 4ms = 4000µs）
 * 2026-10-05 F407VG 迁移新增：与 50Hz 组分 TIM（不同帧率不能共定时器） */
#define PIN_PWM250HZ_TIM        TIM9
#define PIN_PWM250HZ_CH1        TIM_CHANNEL_1   /* PE5 */
#define PIN_PWM250HZ_CH2        TIM_CHANNEL_2   /* PE6 */
#define PIN_PWM250HZ_ARR        1999u
#define PIN_PWM250HZ_US_PER_STEP 2u     /* 4ms 周期 / (ARR+1)=2000 步 */

/* 50Hz PWM 组（TIM5 CH1-3，APB1 定时器 84MHz，PSC=167 ARR=9999；
 * 用途：SG90 舵机脉宽控制，1 step = 2µs，帧长 20ms = 20000µs）
 * 2026-10-05 F407VG 迁移：CH4/PA3 释放（原 4 路舵机拆为 250Hz×2 + 50Hz×3） */
#define PIN_PWM50HZ_TIM         TIM5
#define PIN_PWM50HZ_CH1         TIM_CHANNEL_1   /* PA0 */
#define PIN_PWM50HZ_CH2         TIM_CHANNEL_2   /* PA1 */
#define PIN_PWM50HZ_CH3         TIM_CHANNEL_3   /* PA2 */
#define PIN_PWM50HZ_ARR         9999u
#define PIN_PWM50HZ_US_PER_STEP 2u      /* 20ms 周期 / (ARR+1)=10000 步；
                                             * 2026-10-02 步距 20µs→2µs（舵机指令
                                             * 分辨率 1.8°→0.18°/步） */

/* 蜂鸣器方波组（TIM14 CH1，PA7，APB1 定时器 84MHz；用途：无源蜂鸣器激励，
 * 只用占空比 50%/0% 控制响停，脉宽接口对它无意义）。2026-10-10 新增：
 * ARR 为 CubeMX 默认 65535（PSC=0 → 84M/65536≈1282Hz）——无源蜂鸣器典型
 * 谐振 2.7~4kHz，嫌音调低在 .ioc 改 TIM14 时基并同步本值。
 * ⚠ ARR 为 CubeMX 对账值：改 .ioc 时基必须同步本文件（S5 教训） */
#define PIN_PWM14_TIM           TIM14
#define PIN_PWM14_CH1           TIM_CHANNEL_1   /* PA7（原 DIR2B 脚，DIR2 迁 PC4/PC5） */
#define PIN_PWM14_ARR           65535u

/* ================================ 编码器输入 =============================== */

/* TI12 四倍频，IC Filter = 5；BSP 只出原始增量，换算系数在 robot_config.h
 * （F4 无 AFIO 重映射机制：PA15/PB3/PB4 直接配 AF，无 F1 的 SWJ_CFG 陷阱） */
#define PIN_ENC1_TIM            TIM1    /* PA8/PA9  */
#define PIN_ENC2_TIM            TIM2    /* PA15/PB3 */
#define PIN_ENC3_TIM            TIM3    /* PB4/PB5（2026-09-28 由 PA6/PA7 迁入，随迁移等位保留） */
#define PIN_ENC4_TIM            TIM8    /* PC6/PC7  */

/* 编码器输入引脚对象宏（Gpio_Read 读原始电平，诊断信号通断用） */
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

/* ADC1+ADC2 双同步规则组 + 连续转换 + DMA2_Stream0 循环（32 位打包读 CDR：
 * 低16=ADC1，高16=ADC2；F4 前提 DMAContinuousRequests=ENABLE 已由 .ioc 保证）。
 * 双 rank 等长序列（2026-10-05 F407VG 重构）：rank1=电源 V/I，rank2=关节 J。
 * 原件：rank1 = IN4(PA4 电流)+IN5(PA5 电压)；rank2 = IN12(PC2 J0)+IN13(PC3 J1)。
 * F4 上 MULTI≠0 会屏蔽 ADC3 独立启动（勘误#6），ADC3 退役、PC2/PC3 改挂
 * ADC1/ADC2 rank2。通道号由 .ioc 序列配置，BSP 不再经 cfg.Channel 重配 */
#define PIN_ADC_BATTERY_ADC     ADC1            /* PA4 电流 */
#define PIN_ADC_BATTERY_CH      ADC_CHANNEL_4
#define PIN_ADC2_SYNC_CH        ADC_CHANNEL_5   /* PA5 电压（ADC2 同步采样） */
#define PIN_ADC1_JOINT_CH       ADC_CHANNEL_12  /* PC2 J0（ADC1 rank2） */
#define PIN_ADC2_JOINT_CH       ADC_CHANNEL_13  /* PC3 J1（ADC2 rank2） */

/* ================================== 串口 ================================== */

/* 调试日志口 USART2（TX=PD5 / RX=PD6，115200，纯中断收发，无 DMA）。
 * 2026-10-05 随 F407VG 迁移由 UART4/PC10,11 改挂：板上 UART4 阻塞式判别
 * 未通而 USART2 直通（PC10 引脚占用/位置存疑，UART4 在 CubeMX 仍保留备用）。
 * 2026-10-07 随 F407VE 换板 RX 由 PA3 迁至 PD6。
 * ⚠ 注意：CubeMX 句柄是 huart2（extern 自 usart.h）。
 * CMSIS 的 `USART2` 宏是外设寄存器块指针（USART_TypeDef*），与句柄是两回事，
 * 严禁强转后当句柄传给 HAL——同理 TIM4/ADC1/SPI2 等实例宏都只是寄存器块，
 * HAL 句柄一律用 htim4/hadc1/hspi2（各外设头文件里 extern）。 */

#endif /* BSP_PIN_H */
