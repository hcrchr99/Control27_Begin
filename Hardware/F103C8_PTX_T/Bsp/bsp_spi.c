/**
 * @file    bsp_spi.c
 * @brief   SPI1 全双工字节级收发实现
 *
 * CubeMX 基线：主模式 8bit、CPOL=0/CPHA=1EDGE、软件 NSS、分频 8。
 * ⚠ SPI1 挂 APB2（72MHz），实际时钟 = 72/16 = 4.5MHz（与车端 SPI2 分频 8@APB1 同为 4.5MHz，
 *   2026-09-29 已勘误）；4.5MHz 远低于 nRF24L01+ 的 10MHz 上限，无需改分频。
 */
#include "bsp_spi.h"
#include "bsp_pin.h"
#include "bsp_gpio.h"
#include "robot_config.h"
#include "spi.h"
#include "stm32f1xx_hal.h"

bool Spi_Init(void)
{
    if (!Spi_IsReady())
    {
        return false;
    }
    Spi_Csn(false);     /* 空闲态：未选中（高） */
    Spi_Ce(false);      /* 空闲态：拉低 */
    return true;
}

bool Spi_Transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    /* 接收黑洞：HAL 要求收发指针均有效；SPI1 单消费者（bsp_spi.h 契约），无竞争 */
    static uint8_t s_rx_sink;

    if (tx == NULL || len == 0u || !Spi_IsReady())
    {
        return false;
    }
    return HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)tx,
                                   (rx != NULL) ? rx : &s_rx_sink,
                                   len, ROBOT_SPI_TIMEOUT_MS) == HAL_OK;
}

uint8_t Spi_TransferByte(uint8_t val)
{
    uint8_t rx = 0u;
    (void)Spi_Transfer(&val, &rx, 1u);
    return rx;
}

void Spi_Csn(bool active)
{
    /* 低有效：active=true 拉低 = 选中 */
    if (active)
    {
        Gpio_Reset(PIN_WL_CSN);
    }
    else
    {
        Gpio_Set(PIN_WL_CSN);
    }
}

void Spi_Ce(bool active)
{
    /* 高有效：active=true 拉高 = 使能 */
    if (active)
    {
        Gpio_Set(PIN_WL_CE);
    }
    else
    {
        Gpio_Reset(PIN_WL_CE);
    }
}

bool Spi_IsReady(void)
{
    /* hspi1 是 HAL 句柄（spi.h extern）；SPI1 宏只是寄存器块指针，只用于比对 */
    return (hspi1.Instance == SPI1) && (hspi1.State == HAL_SPI_STATE_READY);
}
