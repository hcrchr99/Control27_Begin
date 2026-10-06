cmake_minimum_required(VERSION 3.22)
project(RM_Begin C ASM)

#
# 仓库根 = CMake 工程入口（对齐 control-2026 结构，CLion 直接打开 RM_Begin 即可加载）
#
# 完整的 CubeMX 工程在 Hardware/F407VG/，作为子目录引入：
#   - CubeMX 重新生成代码只覆盖 Hardware/F407VG/ 下文件，本文件不会被触碰
#   - F407VG/CMakeLists.txt 内部用 CMAKE_CURRENT_LIST_DIR 定位分层目录，
#     无论从仓库根还是从 F407VG 单独配置均可正常工作
#   - 工具链由根目录 CMakePresets.json 指定（toolchainFile → F407VG/cmake/gcc-arm-none-eabi.cmake）
#
# 2026-10-05 F407VG 迁移：BSP 共享层已切 F4（stm32f4xx_hal.h + F4 语义），
# F103RC 目标随之封存——旧车端工程如需重编，checkout 本切换提交之前的
# main（或 F407_SW 的父提交）；其 CMakeLists 保留原样供查阅。
# add_subdirectory(Hardware/F103RC)
add_subdirectory(Hardware/F407VG)
