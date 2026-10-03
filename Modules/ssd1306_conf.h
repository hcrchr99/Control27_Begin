/**
 * @file    ssd1306_conf.h
 * @brief   SSD1306 库移植配置（RM_Begin S7）
 *
 * 来源模板：afiskon/stm32-ssd1306 ssd1306_conf_template.h（MIT）。
 * 相对模板的改动：
 *  - 家族宏不再需要（ssd1306.h 已去 HAL include，传输走 Bsp/bsp_iic）；
 *  - 地址取 robot_config.h 的 ROBOT_OLED_I2C_ADDR（7 位格式，0x3C），
 *    左移由 bsp_iic 统一处理，换 0x3D 屏只改 robot_config.h；
 *  - SSD1306_DELAY_MS → osDelay（Init 仅在任务上下文调用——Modules 层
 *    RTOS 例外，同 Modules/remote/remote.h 的规划授权先例）；
 *  - 字体裁剪：仅 6x8 / 7x10（flash 预算，BSP 规划 §八）。
 */
#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

#include "robot_config.h"
#include "cmsis_os.h"

// Choose a bus
#define SSD1306_USE_I2C
//#define SSD1306_USE_SPI

// I2C Configuration（7 位地址，bsp_iic 内部左移为 HAL 8 位格式）
#define SSD1306_I2C_ADDR        ROBOT_OLED_I2C_ADDR

// 上电稳定等待 / 初始化节拍（仅任务上下文调用链）
#define SSD1306_DELAY_MS(ms)    osDelay(ms)

// Mirror the screen if needed
// #define SSD1306_MIRROR_VERT
// #define SSD1306_MIRROR_HORIZ

// Set inverse color if needed
// #define SSD1306_INVERSE_COLOR

// Include only needed fonts（字体按需编译，flash 预算）
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10

// The width of the screen can be set using this define. Default 128.
// #define SSD1306_WIDTH           128

// If your screen horizontal axis does not start in column 0 you can
// use this define to adjust the horizontal offset
// #define SSD1306_X_OFFSET

// The height can be 32, 64 or 128. Default 64.
// #define SSD1306_HEIGHT          64

#endif /* __SSD1306_CONF_H__ */
