/**
 * @file    s0_bringup_test.c
 * @brief   S0 验收临时代码（验收后删除）：心跳打印 + 串口回显
 *
 * 覆盖 freertos.c 的 __weak StartApp_Daemon_Task（osPriorityLow，栈 768 word）。
 * 验收标准（BSP层开发规划 六.S0）：printf 走 UART4 输出正常。
 * 烧录后串口助手 115200-N-8-1 连 UART4：应见 500ms 心跳，敲入字符有回显。
 */
#include "cmsis_os.h"
#include <stdio.h>
#include "main.h"
#include "bsp_sys.h"
#include "bsp_log.h"

void StartApp_Daemon_Task(void const *argument);

void StartApp_Daemon_Task(void const *argument)
{
    (void)argument;
    uint32_t beat = 0;

    printf("[S0] bringup start, sysclk=%lu Hz\r\n", (unsigned long)SystemCoreClock);

    for (;;)
    {
        /* 存活证据：LED1 1Hz 翻转。灯闪 = 调度器与任务链路正常，
         * 此时串口仍无输出即可锁定硬件/引脚问题 */
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

        printf("[S0] heartbeat #%lu, t=%lu ms\r\n",
               (unsigned long)++beat, (unsigned long)Bsp_GetMs());

        uint8_t rx[32];
        uint16_t n = Log_Poll(rx, sizeof(rx));
        for (uint16_t i = 0; i < n; i++)
        {
            printf("[S0] rx 0x%02X '%c'\r\n", rx[i],
                   (rx[i] >= 0x20 && rx[i] < 0x7F) ? rx[i] : '.');
        }
        if (n > 0)
        {
            printf("[S0] echo %u byte(s)\r\n", (unsigned)n);
        }
        osDelay(500);
    }
}
