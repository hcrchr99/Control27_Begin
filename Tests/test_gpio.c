/**
 * @file    test_gpio.c
 * @brief   S1 测试项：按键 EXTI → LED1 / 蜂鸣器（bsp_gpio 验收语义复原）
 *
 * 验收标准（commit 3b5351c）：KEY1 按下亮 LED1 + 蜂鸣器短鸣，KEY2 按下熄灭。
 * 复原说明：原 S1 临时验收代码在提交前删除未入库，本文件按 commit message
 * 记录的验收语义重写；EXTI 回调只置标志（bsp_gpio 回调契约），消抖/动作在
 * poll 侧做。
 */
#include "test_bench.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "bsp_pin.h"
#include "bsp_sys.h"
#include "bsp_log.h"

static volatile bool s_key1_evt;    /* EXTI 回调置位，poll 侧消费 */
static volatile bool s_key2_evt;
static uint32_t      s_beep_off_ms; /* 短鸣截止时刻，0 = 不在鸣响 */

static void Key1Callback(void) { s_key1_evt = true; }
static void Key2Callback(void) { s_key2_evt = true; }

void Test_Gpio_Init(void)
{
    Exti_Attach(PIN_KEY1_GPIO_PIN, Key1Callback);
    Exti_Attach(PIN_KEY2_GPIO_PIN, Key2Callback);
    Gpio_Reset(PIN_LED1);
    Pwm_SetDuty(PWM_TIM14_CH1, 0.0f);       /* 蜂鸣器静音 */
    Log_Printf("[T-GPIO] KEY1=亮LED1+短鸣 KEY2=灭\r\n");
}

void Test_Gpio_Poll(void)
{
    if (s_key1_evt)
    {
        s_key1_evt = false;
        Gpio_Set(PIN_LED1);
        Pwm_SetDuty(PWM_TIM14_CH1, 0.5f);   /* 无源蜂鸣器：50% 方波鸣响（音调=TIM14 时基） */
        s_beep_off_ms = Bsp_GetMs() + 100u; /* 短鸣 100ms */
        Log_Printf("[T-GPIO] KEY1 down\r\n");
    }
    if (s_key2_evt)
    {
        s_key2_evt = false;
        Gpio_Reset(PIN_LED1);
        Pwm_SetDuty(PWM_TIM14_CH1, 0.0f);
        Log_Printf("[T-GPIO] KEY2 down\r\n");
    }
    if (s_beep_off_ms != 0u &&
        (int32_t)(Bsp_GetMs() - s_beep_off_ms) >= 0)
    {
        Pwm_SetDuty(PWM_TIM14_CH1, 0.0f);
        s_beep_off_ms = 0u;
    }
}
