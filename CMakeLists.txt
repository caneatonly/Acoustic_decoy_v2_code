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
set(CMAKE_PROJECT_NAME Acoustic_decoy_v2)


# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# moved to after project() to avoid warning

# Core project settings
project(${CMAKE_PROJECT_NAME})


# Enable CMake support for AM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# FreeRTOS kernel Library build setup

set(KERNEL_SRC_DIR "${CMAKE_SOURCE_DIR}/Middlewares/FreeRTOS/Source")
set(FREERTOS_PORT_DIR "${KERNEL_SRC_DIR}/portable/GCC/ARM_CM3")

set(FREERTOS_KERNEL_SRCS
    ${KERNEL_SRC_DIR}/croutine.c
    ${KERNEL_SRC_DIR}/event_groups.c
    ${KERNEL_SRC_DIR}/list.c
    ${KERNEL_SRC_DIR}/queue.c
    ${KERNEL_SRC_DIR}/stream_buffer.c
    ${KERNEL_SRC_DIR}/tasks.c
    ${KERNEL_SRC_DIR}/timers.c
    ${FREERTOS_PORT_DIR}/port.c
    ${FREERTOS_PORT_DIR}/../../MemMang/heap_4.c
)

add_library(freertos_kernel STATIC)
target_sources(freertos_kernel PRIVATE ${FREERTOS_KERNEL_SRCS})
target_include_directories(freertos_kernel PUBLIC
    ${KERNEL_SRC_DIR}/include
    ${FREERTOS_PORT_DIR}
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Controls/Inc
)





# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Peripherials/Src/bsp_usart.c
    Peripherials/Src/im948_CMD.c
    Peripherials/Src/bsp_io.c
    Peripherials/Src/MS5837_lib.c
    Peripherials/Src/baro_adc.c
    Controls/Src/valve_ctrl.c
    Controls/Src/depth_estimator.c
    Controls/Src/mission_manager.c
    Controls/Src/depth_cascaded_ctrl.c
    Controls/Src/mission_exec.c
    Controls/Src/balloon_state.c
    Controls/Src/telemetry.c
    Controls/Src/time_utils.c
    Controls/Src/actuators.c
    Controls/Src/sensors_data_get.c
    Controls/Src/control_loop.c
    Core/Src/freertos_hooks.c
    Controls/Src/control_tasks.c
    Controls/Src/console.c
    Controls/Src/tasks_runtime.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Peripherials/Inc
    Controls/Inc
    Core/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)
# 添加浮点数支持的链接选项
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -Wl,-u,_printf_float
    -Wl,-u,_scanf_float
    -lm
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    freertos_kernel
    # Add user defined libraries
)
