cmake_minimum_required(VERSION 3.16)

project(motor2019)

set(RUN_EXEC "${PROJECT_NAME}.elf")

include(../../cmake/options.cmake)
include(../libs/libs.cmake)
include(../shared/shared.cmake)

include_directories(
    ${cmsis_core_INCLUDE_DIRS}
    ${shared_INCLUDE_DIRS}
    ${math_INCLUDE_DIRS}
    ${dsp_INCLUDE_DIRS}
    ${f0xx_INCLUDE_DIRS}
)

add_compile_definitions(
    STM32F0XX
    ARM_MATH_CM0
    __FPU_PRESENT=0
    CORTEX_USE_FPU=0
)

add_compile_options(
    -mcpu=cortex-m0
    -mfloat-abi=soft
)

add_link_options(
    -mcpu=cortex-m0
    -mfloat-abi=soft
)

add_executable(${RUN_EXEC}
    adc.c
    dribbler.c
    dribbler_ctrl.c
    dribbler_motor.c
    drive.c
    drive_ctrl.c
    drive_motor.c
    main.c
    usart1.c
    
    ${shared_SOURCES}
    ${math_SOURCES}
    ${f0xx_SOURCES}
)

target_link_options(${RUN_EXEC} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/run.ld
    -Wl,-Map=${RUN_EXEC}.map
)

include(../../cmake/post_build.cmake)
