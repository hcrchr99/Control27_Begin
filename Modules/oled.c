/**
 * @file    oled.c
 * @brief   OLED 显示模块实现：ssd1306 器件库门面 + 行尾擦除 + 刷新自愈
 *
 * 分层自查（架构 v1.1 规则 1/2，commit 前对照）：本文件只 include 同层
 * ssd1306* 与 Bsp 的 robot_config.h；I²C 访问全部经 ssd1306.c → bsp_iic，
 * 不直接触碰 HAL 句柄（Modules 层不见外设寄存器）。
 */
#include "oled.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "robot_config.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* 默认排版字体：7x10（128/7=18 字符行宽 × 64/10=6 行），测试台与 sense_task 共用 */
#define OLED_DEFAULT_FONT   Font_7x10
#define OLED_PRINTF_BUF     64

/* Oled_Refresh 连败计数（成功清零；达阈值触发器件重初始化，见 oled.h） */
static uint8_t s_refresh_fail_run;

bool Oled_Init(void)
{
    s_refresh_fail_run = 0u;
    return ssd1306_Init();      /* 含上电稳定 + 清屏 + 首刷，清单见 oled.h 头部 */
}

void Oled_Printf(uint8_t x, uint8_t y, const char *fmt, ...)
{
    char buf[OLED_PRINTF_BUF];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0)
    {
        return;
    }

    ssd1306_SetCursor(x, y);
    (void)ssd1306_WriteString(buf, OLED_DEFAULT_FONT, White);

    /* 行尾残留擦除：从文本尾到行宽填背景色——数值变短（如 12345mV→987mV）
     * 时旧字符不清自除。文本写满整行则无残留（x_tail 出界即跳过）；
     * 字符数按实际写入串计（WriteString 越界截断的场景必然已到行尾） */
    uint16_t x_tail = (uint16_t)x + (uint16_t)strlen(buf) * OLED_DEFAULT_FONT.width;
    if (x_tail < SSD1306_WIDTH)
    {
        uint16_t y_end = (uint16_t)y + OLED_DEFAULT_FONT.height - 1u;
        if (y_end >= SSD1306_HEIGHT)
        {
            y_end = SSD1306_HEIGHT - 1u;
        }
        ssd1306_FillRectangle((uint8_t)x_tail, y, SSD1306_WIDTH - 1u, (uint8_t)y_end, Black);
    }
}

bool Oled_Refresh(void)
{
    if (!ssd1306_UpdateScreen())
    {
        if (++s_refresh_fail_run >= (uint8_t)ROBOT_OLED_REINIT_AFTER_FAILS)
        {
            /* 拔插排线 = OLED 模块掉电，寄存器态全丢（坑#5）：重发 init 序列
             * （含充电泵 0x8D 0x14）；成败由下一次 Refresh 检验，此处不阻塞判定 */
            s_refresh_fail_run = 0u;
            (void)ssd1306_Init();
        }
        return false;
    }
    s_refresh_fail_run = 0u;
    return true;
}
