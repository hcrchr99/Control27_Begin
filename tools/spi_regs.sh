#!/usr/bin/env bash
# SPI2/nRF24 引脚寄存器直读：J-Link Commander mem32 活体读取（不停核，结尾 g 续跑）。
# 用途：SPI 总线读异常（如 STATUS=0x7F）时，区分"MCU 侧配置损坏"与"引脚↔模块物理段"。
# 用法: tools/spi_regs.sh
# 地址依据 STM32F407 内存映射：
#   SPI2 CR1/CR2/SR = 0x40003800/04/08（APB1，bsp_spi 用 5.25MHz 主模式）
#   GPIOB MODER=0x40020400  AFRH=0x40020424  IDR=0x40020410  ODR=0x40020414
#     PB13=SCK PB14=MISO PB15=MOSI（AF5）；IDR bit14 = MISO 空闲电平
#   GPIOC ODR=0x40020814（bit12 = nRF24_CSN，空闲必须 1）
#   GPIOD ODR=0x40020C14（bit2 = nRF24_CE，空闲必须 0）
set -e
cd "$(dirname "$0")/.."
JLINK="C:/Program Files/SEGGER/JLink/JLink.exe"
[ -f "$JLINK" ] || JLINK="C:/Program Files (x86)/SEGGER/JLink_V722b/JLink.exe"

mkdir -p build
cat > build/spiregs.jlink <<EOF
mem32 0x40003800 3
mem32 0x40020400 1
mem32 0x40020424 1
mem32 0x40020410 1
mem32 0x40020814 1
mem32 0x40020C14 1
qc
EOF
"$JLINK" -device STM32F407VG -if SWD -speed 4000 -NoGui 1 -ExitOnError 1 \
         -CommandFile build/spiregs.jlink
