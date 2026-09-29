cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME F103RC)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
# 分层代码位于 CubeMX 工程之外(与 Hardware 同级), 对齐 control-2026 结构:
#   RM_Begin/Bsp  RM_Begin/Modules  RM_Begin/UserApp
# 用 CMAKE_CURRENT_LIST_DIR 定位(而非 CMAKE_SOURCE_DIR), 从仓库根或 F103RC
# 单独配置均正确; GLOB_RECURSE 自动收集子目录, 新增 .c 文件无需改本文件
file(GLOB_RECURSE BSP_SOURCES    CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/../../Bsp/*.c")
file(GLOB_RECURSE MODULE_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/../../Modules/*.c")
file(GLOB_RECURSE USERAPP_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/../../UserApp/*.c")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    ${BSP_SOURCES}
    ${MODULE_SOURCES}
    ${USERAPP_SOURCES}
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    "${CMAKE_CURRENT_LIST_DIR}/../../Bsp"
    "${CMAKE_CURRENT_LIST_DIR}/../../Modules"
    "${CMAKE_CURRENT_LIST_DIR}/../../Modules/remote"
    "${CMAKE_CURRENT_LIST_DIR}/../../UserApp"
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
