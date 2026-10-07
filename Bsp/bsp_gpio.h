/**
 * @file    bsp_gpio.h
 * @brief   GPIO 电平读写 + EXTI 回调注册（BSP 层唯一的行为注册点）
 *
 * 设计边界（BSP层开发规划 v1.1 四.2 / 实例化边界分析）：
 *  - GPIO 本体是单实例外设，Gpio_* 用模块级函数即可；
 *  - EXTI 是唯一保留"注册"的地方：中断产生点在 BSP、处理行为属于上层
 *    （Modules/remote 注入 nRF24 IRQ、UserApp 注入按键），而 BSP 禁止
 *    include 上层头文件——函数指针注册是跨层注入的唯一干净通道。
 *    仅此一处，不引入 Instance/Register 框架：一线一回调源、初始化期
 *    固定注册，用不到运行期生命周期管理。
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_pin.h"

/**
 * @brief EXTI 回调函数指针
 *
 * ⚠ 回调契约（违反即不可预期的运行时故障，BSP 无法设防）：
 *  - 运行在 ISR 上下文（NVIC 优先级 5，恰为 RTOS 可调用边界），
 *    只允许：置 volatile 标志 / xTaskNotifyFromISR 等 FromISR 操作；
 *  - 禁止：Log_Printf（日志留给任务侧）、阻塞、非 FromISR 的 RTOS API、
 *    malloc、长循环（格式化/打印一律放任务侧）；
 *  - 消抖不在回调做（规划明确），任务侧处理。
 */
typedef void (*ExtiCallback_t)(void);

/**
 * @brief  注册 EXTI 线回调（仅允许初始化阶段调用，不在运行期动态改挂）
 * @param  gpio_pin GPIO_PIN_x 宏（单 bit），即 EXTI 线号：GPIO_PIN_5 ⇔ Line5。
 *         直接复用 bsp_pin.h 现有宏（KEY1_Pin 等），不另设线号宏组。
 * @param  cb 回调；传 NULL 为注销。重复注册为覆盖语义，后注册生效。
 * @retval true 注册成功；false 传入非法（0 或多 bit）。
 * @note   一线一回调是契约：需要区分多个来源就注册不同的回调，
 *         不要在回调内做来源分支（KEY1/KEY2/nRF24_IRQ 同挂 EXTI15_10 一条线，
 *         2026-10-07 F407VE 换板后 IRQ 已与按键同线，坑 #9 延伸）。
 */
bool Exti_Attach(uint16_t gpio_pin, ExtiCallback_t cb);

/* 电平写：拆 Set/Reset 两个函数，单方向控制脚（LED/蜂鸣器/CSN/CE/STBY）
 * 用起来比布尔参数直观。pin 用 bsp_pin.h 的对象宏（PIN_LED1 等）。 */
void Gpio_Set(GpioPin_t pin);
void Gpio_Reset(GpioPin_t pin);

/* 电平读：返回 true = 高电平（按键上拉输入未按下即 true） */
bool Gpio_Read(GpioPin_t pin);

#endif /* BSP_GPIO_H */
