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

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    Peripherials/Src/bsp_usart.c
    Peripherials/Src/im948_CMD.c
    Peripherials/Src/sensor_process.c
    Peripherials/Src/MS5837_lib.c
    Peripherials/Src/baro_adc.c
    Peripherials/Src/control_algorithm.c
    Controls/Src/depth_estimator.c
    Controls/Src/mission_manager.c
    Controls/Src/depth_cascaded_ctrl.c
    Controls/Src/mission_exec.c
    Controls/Src/balloon_state.c
    Controls/Src/safety_monitor.c
    Controls/Src/telemetry.c
    Controls/Src/param_registry.c
    Controls/Src/actuators.c
    Controls/Src/control_loop.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    Peripherials/Inc
    Controls/Inc

)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)
# 添加浮点数支持的链接选项
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
    -lm
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
