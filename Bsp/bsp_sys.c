/**
 * @file    bsp_sys.c
 * @brief   DWT 微秒计时/延时 + 时钟自检
 */
#include "bsp_sys.h"
#include "bsp_pin.h"
#include "stm32f1xx_hal.h"

#define BSP_SYS_EXPECT_HZ   72000000u   /* HSE 8MHz x PLL9，见 main.c SystemClock_Config */

void Bsp_Init(void)
{
    /* 时钟自检：SystemCoreClock 与 RCC 实际时钟源不符则停机闪烁 LED2（PC13，低电平亮）。
     * 调用点在 MX_GPIO_Init 之后，GPIO 已可用。 */
    SystemCoreClockUpdate();
    if (SystemCoreClock != BSP_SYS_EXPECT_HZ ||
        (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
    {
        while (1)
        {
            HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
            for (volatile uint32_t i = 0; i < 720000u; i++) { __NOP(); }
        }
    }

    /* 使能 DWT 周期计数器（Cortex-M3 内核外设，无需 HAL 模块） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t Bsp_GetUs(void)
{
    /* 64 位换算避免 CYCCNT 直接除造成精度损失；返回值回绕约 71.6 分钟 */
    return (uint32_t)((uint64_t)DWT->CYCCNT * 1000000u / SystemCoreClock);
}

uint32_t Bsp_GetMs(void)
{
    return HAL_GetTick();
}

void Bsp_DelayUs(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks)
    {
        __NOP();
    }
}
