#!/usr/bin/env bash
# 自动化烧录：F407VG.elf → J-Link Commander → 复位运行
# 用法: tools/flash.sh
# ⚠ 为什么不是 OpenOCD：本机 J-Link 被 Segger 官方驱动独占（Ozone 在用），
# sysprogs OpenOCD 走 libusb 抢不到设备；JLink.exe 与 Ozone 共存无冲突。
set -e
cd "$(dirname "$0")/.."
ELF="build/Debug/Hardware/F407VG/F407VG.elf"
JLINK="C:/Program Files/SEGGER/JLink/JLink.exe"
[ -f "$ELF" ] || { echo "ELF 不存在，先编译: cmake --build --preset Debug"; exit 1; }
[ -f "$JLINK" ] || JLINK="C:/Program Files (x86)/SEGGER/JLink_V722b/JLink.exe"

mkdir -p build
cat > build/flash.jlink <<EOF
r
h
loadfile $ELF
r
g
qc
EOF
"$JLINK" -device STM32F407VG -if SWD -speed 4000 -NoGui 1 -ExitOnError 1 \
         -CommandFile build/flash.jlink
