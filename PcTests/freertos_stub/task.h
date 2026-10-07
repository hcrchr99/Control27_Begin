/**
 * @file    task.h
 * @brief   FreeRTOS task.h 的 host 替身（仅 PC 单测）——关调度两函数在
 *          host 上是无内核的空操作，由 freertos_stub.c 提供定义
 */
#ifndef FREERTOS_STUB_TASK_H
#define FREERTOS_STUB_TASK_H

void vTaskSuspendAll(void);
void xTaskResumeAll(void);

#endif /* FREERTOS_STUB_TASK_H */
