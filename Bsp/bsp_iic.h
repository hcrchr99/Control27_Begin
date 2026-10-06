/**
 * @file    bsp_iic.h
 * @brief   I²C2 阻塞读写：地址参数化对称接口 + 超时 + 连续失败总线恢复
 *
 * 设计（BSP层开发规划 四.4 / 架构 v1.1 §3.2 / Modules 坑#11，2026-10-03 S7）：
 *  - 仅片上 I²C2（PB10/PB11，100kHz 标准模式，.ioc 基线零改动），阻塞 + 超时，
 *    无中断无 DMA：整页 129B @100kHz ≈ 12ms，ROBOT_IIC_TIMEOUT_MS=20 余量充足；
 *  - ⚠ 地址参数化对称接口是 S7 硬性验收点（Modules 坑#11：I²C2 扩展插针预留
 *    IMU 0x68/0x6A/0x6B；BSP 规划行125 旧草案 Iic_Write(buf,len) 不带地址是
 *    缺陷，本文件即该勘误的落实）。addr 一律 7 位格式，内部左移成 HAL 的
 *    8 位格式——调用方写器件手册原值（0x3C/0x68），不再操心 <<1；
 *  - WriteReg/ReadReg（HAL_Mem 语义）服务"寄存器/控制字节"访问：SSD1306 的
 *    reg=0x00 命令流 / 0x40 数据流（SSD1306 手册 §8.1.5），IMU 的 reg=寄存器
 *    指针，两类器件共用同一薄封装；裸 Write/Read 留给无寄存器语义的器件；
 *  - 坑#10：OLED 排线接触不良会拉死 I²C2——全部 API 带超时；连续失败
 *    ROBOT_IIC_RECOVER_N 次触发总线恢复（HAL_I2C_DeInit→Init，Msp 层
 *    CubeMX 已生成），禁止死等。拔插排线场景（从机消失又回来）该策略
 *    即自愈；若出现从机 SDA 拉死的场景过不了，再升级"9 时钟 SCL GPIO
 *    释放"（预留升级位，首版不做）；
 *  - 探测失败同样计入连败计数（无伤：恢复只在达阈值时发生，T4 对未插
 *    器件 0x68 探测一次仅 +1）；
 *  - 非并发契约：I²C2 当前仅 Modules/oled 一个消费者，调用方自行保证互斥。
 */
#ifndef BSP_IIC_H
#define BSP_IIC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  就绪校验（幂等）：确认 MX_I2C2_Init 已跑（Instance==I2C2，S0 教训）。
 *         不重跑 MX_I2C2_Init（main.c 已在调度器启动前调用，避免双源真相）。
 * @retval false = hi2c2 未就绪（Instance 非 I2C2）
 */
bool Iic_Init(void);

/**
 * @brief  裸写 len 字节（阻塞，超时 ROBOT_IIC_TIMEOUT_MS）。
 * @param  dev_addr7 7 位从机地址（0x3C 等，内部左移）
 * @retval false = 参数非法 / 未初始化 / 超时 / HAL 错误（连败达阈值触发总线恢复）
 */
bool Iic_Write(uint8_t dev_addr7, const uint8_t *buf, uint16_t len);

/**
 * @brief  裸读 len 字节（阻塞）。地址参数化对称接口的读侧（坑#11）。
 */
bool Iic_Read(uint8_t dev_addr7, uint8_t *buf, uint16_t len);

/**
 * @brief  写器件寄存器/控制字节（HAL_I2C_Mem_Write 语义，8 位寄存器宽）。
 *         SSD1306：reg=0x00 命令流 / 0x40 数据流；IMU：reg=寄存器指针。
 */
bool Iic_WriteReg(uint8_t dev_addr7, uint8_t reg, const uint8_t *buf, uint16_t len);

/**
 * @brief  读器件寄存器（HAL_I2C_Mem_Read 语义，写寄存器指针+重复起始读）。
 */
bool Iic_ReadReg(uint8_t dev_addr7, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief  器件在线探测（发地址收 ACK，不写任何数据）。
 * @note   用于 T0 上电自检与 T4 地址参数化验收（对未插器件地址应返回 false）。
 */
bool Iic_IsDeviceReady(uint8_t dev_addr7);

/**
 * @brief  诊断：累计总线恢复次数（T3 拔插自愈的旁证——拔插期间应增长）
 */
uint32_t Iic_GetRecoverCount(void);

#endif /* BSP_IIC_H */
