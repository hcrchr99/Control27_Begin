/**
 * @file    bsp_spi.h
 * @brief   SPI2 全双工字节级收发 + 无线模块控制脚（CSN/CE）电平封装
 *
 * 设计边界（BSP层开发规划 v1.1 四.6 / 实例化边界三问）：
 *  - 是否多实例？否（SPI2 仅挂无线模块）；是否运行期绑定？否（编译期固定）；
 *    是否跨层注入行为？否（收包行为经 bsp_gpio 的 Exti_Attach 注册，不在本层）。
 *    结论：模块级函数 + 静态状态，不做 Instance/Register 框架；
 *  - 只出字节级 API 与片选/使能电平，包级/器件逻辑全部在 Modules/remote（规则 2：
 *    命名仅外设概念，CSN/CE 以"选中/使能"语义呈现，无器件名）。
 */
#ifndef F103RC_BSP_SPI_H
#define F103RC_BSP_SPI_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief  初始化自检 + 强制安全默认电平。
 *         不重跑 MX_SPI2_Init（main.c 已在调度器启动前调用，避免双源真相），
 *         仅校验 hspi2 并强制 CSN=高（未选中）/ CE=低（空闲）。
 * @retval false = hspi2 未就绪（Instance 非 SPI2 或状态非 READY）
 */
bool Spi_Init(void);

/**
 * @brief  全双工收发 len 字节（阻塞，超时 ROBOT_SPI_TIMEOUT_MS，见 robot_config.h）
 * @param  tx 发送缓冲，不可为 NULL；rx 接收缓冲，可为 NULL（接收内容丢弃）
 * @retval false = 参数非法 / hspi2 未就绪 / 超时 / HAL 错误
 * @note   非并发契约：SPI2 仅 remote 服务任务一个消费者，调用方自行保证互斥。
 */
bool Spi_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len);

/**
 * @brief  单字节全双工：发指令的同时收回应答字节（寄存器访问的高频路径）。
 * @retval 传输失败时返回 0x00——上层以状态回读校验兜底（Nrf24_Configure 回读校验）。
 */
uint8_t Spi_TransferByte(uint8_t val);

/**
 * @brief  片选（低有效）：true = 选中拉低，false = 释放拉高。
 * @note   空闲态必须为高（总线规约）；CSN 拉低到首字节之间的建立时间
 *         由调用方（Modules/remote）用 Bsp_DelayUs 控制，本层不加固定延时。
 */
void Spi_Csn(bool active);

/**
 * @brief  无线模块使能脚（高有效）：true = 使能拉高，false = 拉低。
 * @note   极性差异（CSN 低有效 / CE 高有效）在本层消化，上层只见使能语义。
 */
void Spi_Ce(bool active);

/**
 * @brief  诊断：hspi2 就绪且 Instance==SPI2（CMSIS 实例宏≠HAL 句柄，S0 教训）
 */
bool Spi_IsReady(void);

#endif /* F103RC_BSP_SPI_H */
