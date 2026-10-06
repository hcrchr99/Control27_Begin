/**
 * @file    bsp_iic.c
 * @brief   I²C2 阻塞读写实现：超时 + 连败总线恢复（坑#10），地址 7 位入参
 *
 * CubeMX 基线（i2c.c）：I2C2 100kHz 标准模式、AF 开漏、无中断无 DMA——
 * S7 零 .ioc 改动，本文件只做 API 封装与恢复策略，不碰外设寄存器
 * （对比 bsp_adc 的 EXTTRIG 补位：I²C 无 HAL 缺口，无需运行期补配）。
 */
#include "bsp_iic.h"
#include "bsp_log.h"
#include "robot_config.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"

static bool     s_ready = false;
static uint32_t s_fail_run = 0u;    /* 连续失败计数（成功清零） */
static uint32_t s_recover_cnt = 0u; /* 累计总线恢复次数（诊断） */

bool Iic_Init(void)
{
    if (s_ready)
    {
        return true;
    }
    /* MX_I2C2_Init 未跑（Instance 空）则拒绝启动——CMSIS 实例宏≠HAL 句柄（S0 教训） */
    if (hi2c2.Instance != I2C2)
    {
        Log_Printf("[IIC] MX_I2C2_Init 未跑，拒绝启动\r\n");
        return false;
    }
    s_ready = true;
    return true;
}

/**
 * @brief  统一事务出口：成功清零连败计数；连败达 ROBOT_IIC_RECOVER_N 触发
 *         总线恢复（坑#10 规划决议 DeInit→Init，MspDeInit/Init 已由 CubeMX
 *         生成，含 GPIO 复用释放与外设时钟复位）。
 * @note   恢复动作内联在调用者任务上下文执行（毫秒级）， OLED 消费方为
 *         低优先级任务，可承受。
 */
static bool IicSettle(bool ok)
{
    if (ok)
    {
        s_fail_run = 0u;
        return true;
    }
    if (++s_fail_run >= (uint32_t)ROBOT_IIC_RECOVER_N)
    {
        Log_Printf("[IIC] 连续 %d 次事务失败，总线恢复（DeInit→Init）\r\n",
                   (int)ROBOT_IIC_RECOVER_N);
        (void)HAL_I2C_DeInit(&hi2c2);
        (void)HAL_I2C_Init(&hi2c2);
        s_fail_run = 0u;
        s_recover_cnt++;
    }
    return false;
}

/* 7 位地址 → HAL 8 位格式（调用方写器件手册原值，见 bsp_iic.h 设计注释） */
static uint16_t ToAddr8(uint8_t dev_addr7)
{
    return (uint16_t)((dev_addr7 & 0x7Fu) << 1);
}

bool Iic_Write(uint8_t dev_addr7, const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u || !s_ready)
    {
        return false;
    }
    return IicSettle(HAL_I2C_Master_Transmit(&hi2c2, ToAddr8(dev_addr7),
                                             (uint8_t *)(uintptr_t)buf, len,
                                             ROBOT_IIC_TIMEOUT_MS) == HAL_OK);
}

bool Iic_Read(uint8_t dev_addr7, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u || !s_ready)
    {
        return false;
    }
    return IicSettle(HAL_I2C_Master_Receive(&hi2c2, ToAddr8(dev_addr7),
                                            buf, len,
                                            ROBOT_IIC_TIMEOUT_MS) == HAL_OK);
}

bool Iic_WriteReg(uint8_t dev_addr7, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u || !s_ready)
    {
        return false;
    }
    return IicSettle(HAL_I2C_Mem_Write(&hi2c2, ToAddr8(dev_addr7), reg,
                                       I2C_MEMADD_SIZE_8BIT,
                                       (uint8_t *)(uintptr_t)buf, len,
                                       ROBOT_IIC_TIMEOUT_MS) == HAL_OK);
}

bool Iic_ReadReg(uint8_t dev_addr7, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u || !s_ready)
    {
        return false;
    }
    return IicSettle(HAL_I2C_Mem_Read(&hi2c2, ToAddr8(dev_addr7), reg,
                                      I2C_MEMADD_SIZE_8BIT, buf, len,
                                      ROBOT_IIC_TIMEOUT_MS) == HAL_OK);
}

bool Iic_IsDeviceReady(uint8_t dev_addr7)
{
    if (!s_ready)
    {
        return false;
    }
    /* Trials=1：探测语义保持纯粹（重试由调用方决定）；失败计入连败（见头文件注释） */
    return IicSettle(HAL_I2C_IsDeviceReady(&hi2c2, ToAddr8(dev_addr7), 1u,
                                           ROBOT_IIC_TIMEOUT_MS) == HAL_OK);
}

uint32_t Iic_GetRecoverCount(void)
{
    return s_recover_cnt;
}
