/**
 * @file    remoter_app.h
 * @brief   遥控器主任务入口（defaultTask 转发点，见 remoter_app.c）
 */
#ifndef REMOTER_APP_H
#define REMOTER_APP_H

/**
 * @brief  遥控器主任务：初始化（时钟/SPI/无线自检，失败原地闪灯提示）后进入
 *         100Hz 发送主循环（采开关/摇杆 → 建帧 → 发包 + 心跳/统计）。
 * @note   内部自带永不返回的主循环；由 main.c StartDefaultTask USER CODE 5 转发。
 */
void Remoter_Main(void const *argument);

#endif /* REMOTER_APP_H */
