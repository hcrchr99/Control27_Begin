/**
 * @file    freertos_stub.c
 * @brief   关调度两函数的 host 替身：PC 单测单线程跑，无调度器可关
 */
#include "task.h"

void vTaskSuspendAll(void)
{
}

void xTaskResumeAll(void)
{
}
