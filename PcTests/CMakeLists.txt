cmake_minimum_required(VERSION 3.22)
project(AlgorithmPcTests C)

#
# W2.1 algorithm 的 host 编译 PC 单测（Modules 层开发规划 §3.7 / §四）：
# 被测代码 = Modules/algorithm（纯算法，唯一硬件接触点 pid.c 的 bsp_sys
# 由本目录 bsp_sys_stub.c 假时钟替身）。独立于固件构建——固件 CMake 的
# GLOB 只收 Bsp/Modules/UserApp/Tests 四树，本目录不进固件。
#
# 运行：cmake -S PcTests -B PcTests/build && cmake --build PcTests/build
#       ./PcTests/build/pc_tests.exe   （退出码 0 = 全绿）
#

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(ALGO_DIR "${CMAKE_CURRENT_LIST_DIR}/../Modules/algorithm")

add_executable(pc_tests
    "${ALGO_DIR}/pid.c"
    "${ALGO_DIR}/lpf.c"
    "${ALGO_DIR}/ease.c"
    "${ALGO_DIR}/user_lib.c"
    "bsp_sys_stub.c"
    "test_pid.c"
    "test_user_lib.c"
    "test_lpf_ease.c"
    "main.c"
)

target_include_directories(pc_tests PRIVATE
    "${ALGO_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/../Bsp"
    "${CMAKE_CURRENT_LIST_DIR}"
)

if(MSVC)
    target_compile_options(pc_tests PRIVATE /W4)
else()
    target_compile_options(pc_tests PRIVATE -Wall -Wextra)
endif()
