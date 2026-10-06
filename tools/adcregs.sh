#!/usr/bin/env bash
# ADC 寄存器直读：J-Link Commander mem32 活体读取（连接不主动停核，读两轮看
# NDTR 是否在走；结尾 g 保证目标继续运行，便于后续串口观察）。
# 用法: tools/adcregs.sh
# 地址依据 STM32F407 内存映射：
#   ADC 公共 CCR=0x40012304   ADC1 SR/CR1=0x40012000 CR2=0x40012008
#   ADC3 SR=0x40012200  CR2=0x40012208
#   DMA2_Stream0 CR=0x40026410  NDTR=0x40026418  M0AR=0x40026420
#   RCC APB2ENR=0x40023844
#   s_pair_buf=0x200058FC（arm-none-eabi-nm，重编译后需复核）
set -e
cd "$(dirname "$0")/.."
JLINK="C:/Program Files/SEGGER/JLink/JLink.exe"
[ -f "$JLINK" ] || JLINK="C:/Program Files (x86)/SEGGER/JLink_V722b/JLink.exe"

mkdir -p build
cat > build/adcregs.jlink <<EOF
mem32 0x40012304 1
mem32 0x40012000 2
mem32 0x40012008 1
mem32 0x40012200 2
mem32 0x40012208 1
mem32 0x40026410 1
mem32 0x40026418 1
mem32 0x40023844 1
Sleep 500
mem32 0x40026418 1
mem32 0x200058FC 4
g
qc
EOF
"$JLINK" -device STM32F407VG -if SWD -speed 4000 -NoGui 1 \
         -CommandFile build/adcregs.jlink
