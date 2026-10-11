cmake_minimum_required(VERSION 3.22)
project(TwinHarness C)

#
# 数字孪生桥 C 端（PcTwin，host 编译）：被 PcTwin/ws_bridge.py 以子进程驱动。
# 被测源 = 固件同款 rc_cmd + UserApp 业务链（chassis_ctrl/grab_ctrl）原样编译，
# 不是复刻逻辑；FreeRTOS 快照锁由 PcTests 的 freertos_stub 空实现顶替。
# 独立工程——不进固件构建（固件 GLOB 只收 Bsp/Modules/UserApp/Tests 四树），
# 也不混入 PcTests 主程序。
#
# 运行：cmake -S PcTwin -B PcTwin/build && cmake --build PcTwin/build
#       （或直接跑 PcTwin/启动孪生桥.bat）
#

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(ROOT "${CMAKE_CURRENT_LIST_DIR}/..")

add_executable(twin_harness
    "twin_harness.c"
    "${ROOT}/Modules/remote/rc_cmd.c"
    "${ROOT}/UserApp/chassis_ctrl.c"
    "${ROOT}/UserApp/grab_ctrl.c"
    "${ROOT}/PcTests/freertos_stub/freertos_stub.c"
)

target_include_directories(twin_harness PRIVATE
    "${ROOT}/Modules/remote"
    "${ROOT}/UserApp"
    "${ROOT}/Bsp"
    "${ROOT}/PcTests/freertos_stub"
)

if(MSVC)
    target_compile_options(twin_harness PRIVATE /W4)
else()
    target_compile_options(twin_harness PRIVATE -Wall -Wextra)
endif()
