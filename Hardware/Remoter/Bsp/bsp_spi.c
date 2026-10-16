/**
 * @file    bsp_spi.c
 * @brief   SPI2 全双工字节级收发实现（Remoter 移植版）
 *
 * CubeMX 基线：主模式 8bit、CPOL=0/CPHA=1EDGE、软件 NSS。
 * ⚠ SPI2 挂 APB1（36MHz），分频 8 = 实际 4.5MHz（与 PTX 板 SPI1/16、车端 SPI2/8 同速）；
 *   nRF24L01+ 上限 10MHz，余量充足。生成基线曾误配分频 2（=18MHz 超限），已修正。
 */
#include "bsp_spi.h"
#include "bsp_pin.h"
#include "bsp_gpio.h"
#include "robot_config.h"
#include "main.h"
#include "stm32f1xx_hal.h"

/* HAL 句柄在 main.c 定义；main.h 未 extern（CubeMX 未开 CoupleFile），本层自行声明 */
extern SPI_HandleTypeDef hspi2;

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
    /* 接收黑洞：HAL 要求收发指针均有效；SPI2 单消费者（bsp_spi.h 契约），无竞争 */
    static uint8_t s_rx_sink;

    if (tx == NULL || len == 0u || !Spi_IsReady())
    {
        return false;
    }
    return HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx,
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
    /* hspi2 是 HAL 句柄；SPI2 宏只是寄存器块指针，只用于比对 */
    return (hspi2.Instance == SPI2) && (hspi2.State == HAL_SPI_STATE_READY);
}
