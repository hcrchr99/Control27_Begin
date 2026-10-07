/**
 * @file    bsp_gpio.c
 * @brief   GPIO 电平读写 + EXTI 回调注册表实现
 */
#include "bsp_gpio.h"
#include "stm32f4xx_hal.h"

/* ============================ EXTI 回调注册表 ============================= */

/* 按 EXTI 线号 0~15 索引；NULL = 未注册。
 *
 * 并发正确性论证（免临界区）：
 *  - 写入仅发生在初始化阶段（Exti_Attach 契约），ISR 侧只读；
 *  - 表项是 32bit 对齐的函数指针，Cortex-M 上单条 store 原子；
 *  - "初始化期写入、之后只读"的约定排除了理论上的读写竞争窗口。 */
static ExtiCallback_t s_exti_cb[16] = { NULL };

bool Exti_Attach(uint16_t gpio_pin, ExtiCallback_t cb)
{
    /* 须为单 bit：0 或多 bit 都是非法线号 */
    if (gpio_pin == 0u || (gpio_pin & (gpio_pin - 1u)) != 0u)
    {
        return false;
    }
    /* POSITION_VAL：bit 序号 → 线号，Cortex-M4 编译为 RBIT+CLZ，O(1) 无查表 */
    s_exti_cb[POSITION_VAL(gpio_pin)] = cb;
    return true;
}

/* ============================== ISR 分发入口 ============================== */

/* HAL weak 回调的强符号覆盖，F1 HAL 的统一 EXTI 入口
 * （F1/F4 的 HAL_GPIO_EXTI_IRQHandler 均为：清挂起 → 本函数，无上升/下降分体）。
 * KEY1/KEY2 与 nRF24 IRQ 三者共用 EXTI15_10 一条中断线
 * （2026-10-07 F407VE 换板后 IRQ 由 EXTI9_5 的 PC5 迁至 PD10），
 * 本表按线号索引表项，同线多源按 GPIO_Pin 天然区分（坑 #9 延伸）。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* 先拷贝再判空调用：即使表在极端时序下被改写，调用的也是快照 */
    ExtiCallback_t cb = s_exti_cb[POSITION_VAL(GPIO_Pin)];
    if (cb != NULL)
    {
        cb();
    }
}

/* ================================ GPIO 读写 ================================ */

void Gpio_Set(GpioPin_t pin)
{
    HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_SET);
}

void Gpio_Reset(GpioPin_t pin)
{
    HAL_GPIO_WritePin(pin.port, pin.pin, GPIO_PIN_RESET);
}

bool Gpio_Read(GpioPin_t pin)
{
    return HAL_GPIO_ReadPin(pin.port, pin.pin) == GPIO_PIN_SET;
}
