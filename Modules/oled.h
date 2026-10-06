/**
 * @file    oled.h
 * @brief   OLED 显示模块（SSD1306 128×64 I²C）：冻结三 API + SSD1306 手册对照清单
 *
 * 定位（架构 v1.1 §3.3 / Modules 规划 §3.5）：SSD1306 器件驱动 + 排版刷新。
 * 上游 Bsp/bsp_iic；下游 sense_task（W2 接线，S7 由测试台 9=OLED 驱动）。
 * 器件库 ssd1306_*（afiskon/stm32-ssd1306 移植改造，MIT）是本层内部实现，
 * 上层只许 include 本头文件。显示内容四项（电压/电流/功率/链路状态）由
 * 调用方排版，本层只管像素（排版注释归 sense_task 规划）。
 *
 * ========================================================================
 * SSD1306 手册对照清单（datasheet Rev 1.1，坑#5：逐节对照打勾，勿凭记忆）
 * ========================================================================
 *
 * 【§8.1.5 I²C 通信】
 *  ✓ 7 位从机地址 = 0111100 + SA0：SA0=0 → 0x3C（0.96 寸模块默认），
 *    SA0=1 → 0x3D。ROBOT_OLED_I2C_ADDR（robot_config.h）= 0x3C，
 *    左移为 8 位格式由 bsp_iic 统一处理；
 *  ✓ 控制字节 = Co | D/C | 000000：0x00 = 命令流（后续全为命令），
 *    0x40 = 数据流（GDDRAM），0x80 = 单命令后跟新控制字节，
 *    0xC0 = 单数据后跟新控制字节。
 *    实现：Iic_WriteReg(addr, 0x00/0x40, buf, len) 把控制字节当 1 字节
 *    寄存器地址发——线上时序与手册图 8-10 完全一致（START+ADDR+W+CB+data）。
 *
 * 【§8.7 GDDRAM 布局】
 *  ✓ 8 页（PAGE0..7）× 128 列（SEG0..127）；每字节 = 一列上的 8 行像素，
 *    D0 在页顶（COM 侧）、D7 在页底；页 N 覆盖第 8N..8N+7 行；
 *  ✓ ssd1306_DrawPixel：buffer[x + (y/8)*128] 的 bit(y%8)——与手册位序吻合
 *    （移植时逐句核验过，非凭记忆采信 upstream）。
 *
 * 【§10 命令表：初始化序列（对应 ssd1306_Init，逐条登记）】
 *  0xAE        显示关（配置期必关）
 *  0x20 0x00   寻址模式 = 水平（决议见下）
 *  0xB0        页起始地址（页寻址用；水平模式下为无操作，保留）
 *  0xC8        COM 扫描方向（remapped，与 0xA1 配套；MIRROR_VERT 时 0xC0）
 *  0x00/0x10   列地址低 4 位 / 高 4 位（= 0）
 *  0x40        显示起始行 = 0
 *  0x81 0xFF   对比度（最高档，模块实测偏暗再降）
 *  0xA1        段重映射：列 0 → SEG127（与 0xC8 组成正常视角；
 *              MIRROR_HORIZ 时 0xA0）
 *  0xA6        正常显示（非反色）
 *  0xA8 0x3F   复用率 = 64 行
 *  0xA4        输出跟随 RAM 内容（0xA5 = 全亮测试用）
 *  0xD3 0x00   显示偏移 = 0
 *  0xD5 0xF0   时钟分频/振荡频率（上电默认 0x80，upstream 取 0xF0）
 *  0xD9 0x22   预充电周期（Phase1=2 DCLK / Phase2=2 DCLK）
 *  0xDA 0x12   COM 引脚配置（64 行 = 交替 COM 布局）
 *  0xDB 0x20   VCOMH = 0.77 × Vcc
 *  0x8D 0x14   ⚠ 充电泵使能——漏发 = 全黑（模块无内部面板电源，最常见故障）
 *  0xAF        显示开
 *
 * 【决议：寻址模式 = 水平（0x20 0x00）+ 逐页 128B 事务】
 *  水平模式下页/列指针随数据自增：每页一次 128B 写、8 页顺序铺满整帧
 *  （整帧 1024B 不拆事务 @100kHz ≈ 92ms 阻塞）；UpdateScreen 保留 0xB0+i
 *  页命令——水平模式下无效但兼容仅支持页寻址的 SH1106。若换 SH1106 屏
 *  只需改 Init 的寻址模式命令为页寻址（0x20 0x02）。
 *
 * 【上电稳定期】
 *  Power ON → 内部复位 → 等待 VDD/VBAT 稳定后才可发命令；upstream 取
 *  100ms（SSD1306_DELAY_MS = osDelay，仅任务上下文）。待实测可调。
 *
 * ========================================================================
 */
#ifndef OLED_H
#define OLED_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  初始化器件并点亮（幂等，可重复调用）。
 *         序列：100ms 上电稳定（osDelay）→ 对照清单命令流 → 清屏首刷。
 *         失败后可直接重调（命令流自带显示关→开全过程）。
 * @retval false = I²C 事务失败（看 bsp 日志 / Iic_GetRecoverCount 定位）
 * @note   仅任务上下文调用（内部 osDelay）；I²C 阻塞发送对 BelowNormal
 *         的调用方可接受（Modules 规划 §3.5 实现要点①）。
 */
bool Oled_Init(void);

/**
 * @brief  格式化写帧缓冲（只写内存，不触发 I²C——配合 Oled_Refresh 使用）。
 * @param  x,y 文本起点像素坐标（左上角原点，默认字体 7x10）
 * @note   行尾自动擦除：打印后从文本尾到行宽填背景色，数值变短不清自除；
 *         newlib-nano 未启 _printf_float（BSP 坑#14）——格式化串沿用整型
 *         小数化约定（mV/mA/mW），%f 打出来是空的。
 */
void Oled_Printf(uint8_t x, uint8_t y, const char *fmt, ...);

/**
 * @brief  帧缓冲 → 整帧刷屏（8 页 × 128B，@100kHz ≈ 92ms 阻塞）。
 * @retval false = 本次刷新存在 I²C 事务失败（超时/无 ACK）——不挂死，
 *         连续失败 ROBOT_OLED_REINIT_AFTER_FAILS 次后自动重发器件 init
 *         （拔插排线 = 模块掉电寄存器态全丢，坑#5/#10 的自愈路径）。
 */
bool Oled_Refresh(void);

#endif /* OLED_H */
