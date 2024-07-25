cmake_minimum_required(VERSION 3.16)

project(bs2018)

set(RUN_EXEC "${PROJECT_NAME}.elf")
set(BOOT_EXEC "${PROJECT_NAME}_boot.elf")

include(../../cmake/options.cmake)
include(../libs/libs.cmake)
include(../robot/robot.cmake)
include(../shared/shared.cmake)

list(FILTER gui_SOURCES EXCLUDE REGEX top_bar.c|menu.c)
list(FILTER robot_SOURCES EXCLUDE REGEX network.c)

list(APPEND bs2018_SOURCES
    base_station.c
    main.c
    presenter.c
    router.c
    system.c
    shell_commands.c
    
    gui/menu.c
    gui/robot_status.c
    gui/setup.c
    gui/top_bar.c
    
    sys/eth0.c
    sys/rtc.c
    sys/spi3.c
    sys/tim2.c
    sys/tim5.c
    sys/usart1.c
    
    dev/led.c
    dev/shell.c
    dev/sky_fem.c
    dev/sx1280.c
    dev/tft.c
    dev/touch.c
    
    misc/inventory.c
    
    ${shared_SOURCES}
    ${net_SOURCES}
    ${f7xx_SOURCES}
    ${robot_SOURCES}
    ${util_SOURCES}
    ${chibios_SOURCES}
    ${ugfx_SSD1963_SOURCES}
    ${gui_SOURCES}
    ${math_SOURCES}
    ${protobuf_c_SOURCES}
)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    
    ${cmsis_core_INCLUDE_DIRS}
    ${shared_INCLUDE_DIRS}
    ${f7xx_INCLUDE_DIRS}
    ${util_INCLUDE_DIRS}
    ${dsp_INCLUDE_DIRS}
    ${chibios_INCLUDE_DIRS}
    ${ugfx_SSD1963_INCLUDE_DIRS}
    ${gui_INCLUDE_DIRS}
    ${robot_INCLUDE_DIRS}
    ${fatfs_INCLUDE_DIRS}
    ${protobuf_c_INCLUDE_DIRS}
)

add_compile_definitions(
    STM32F7XX
    STM32F767xx
    ARM_MATH_CM7
    __FPU_PRESENT=1
    CORTEX_USE_FPU=1
)

add_compile_options(
    -mcpu=cortex-m7
    -mfloat-abi=hard
    -mfpu=fpv5-d16
)

add_link_options(
    -mcpu=cortex-m7
    -mfloat-abi=hard
    -mfpu=fpv5-d16
    -u_printf_float
    -u_scanf_float
)

# Normal run/application code
add_executable(${RUN_EXEC}
    ${bs2018_SOURCES}
)

target_compile_definitions(${RUN_EXEC} PRIVATE
    ENV_RUN
)

target_link_libraries(${RUN_EXEC} c m)

target_link_options(${RUN_EXEC} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/run.ld
    -L${CMAKE_CURRENT_SOURCE_DIR}
    -L${CMAKE_CURRENT_SOURCE_DIR}/../libs/chibios
    -Wl,-Map=${RUN_EXEC}.map
)

# Bootloader code
add_executable(${BOOT_EXEC}
    main_bootloader.c
    system.c
    
    sys/tim2.c
	sys/usart1.c
	dev/led.c
    
    ${chibios_SOURCES}
    ${util_SOURCES}
    ${f7xx_SOURCES}
)

target_compile_definitions(${BOOT_EXEC} PRIVATE
    ENV_BOOT
)

target_link_libraries(${BOOT_EXEC} c m)

target_link_options(${BOOT_EXEC} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/boot.ld
    -L${CMAKE_CURRENT_SOURCE_DIR}
    -L${CMAKE_CURRENT_SOURCE_DIR}/../libs/chibios
    -Wl,-Map=${BOOT_EXEC}.map
)

include(../../cmake/post_build.cmake)
