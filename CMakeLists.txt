cmake_minimum_required(VERSION 3.22)
project(RM_Begin C ASM)

#
# 仓库根 = CMake 工程入口（对齐 control-2026 结构，CLion 直接打开 RM_Begin 即可加载）
#
# 完整的 CubeMX 工程在 Hardware/F103RC/，作为子目录引入：
#   - CubeMX 重新生成代码只覆盖 Hardware/F103RC/ 下文件，本文件不会被触碰
#   - F103RC/CMakeLists.txt 内部用 CMAKE_CURRENT_LIST_DIR 定位分层目录，
#     无论从仓库根还是从 F103RC 单独配置均可正常工作
#   - 工具链由根目录 CMakePresets.json 指定（toolchainFile → F103RC/cmake/gcc-arm-none-eabi.cmake）
#
add_subdirectory(Hardware/F103RC)
