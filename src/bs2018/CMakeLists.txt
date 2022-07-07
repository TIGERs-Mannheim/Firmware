cmake_minimum_required(VERSION 3.16)

project(bs2018)

set(RUN_EXEC "${PROJECT_NAME}.elf")
set(BOOT_EXEC "${PROJECT_NAME}_boot.elf")

include(../../cmake/options.cmake)
include(../libs/CMakeLists.txt)

list(FILTER gui_SOURCES EXCLUDE REGEX top_bar.c|menu.c)
list(FILTER robot_SOURCES EXCLUDE REGEX network.c)

list(APPEND bs2018_SOURCES
    cli.c
    hub.c
    main.c
    network.c
    presenter.c
    vision.c
    wifi.c
    
    gui/menu.c
    gui/robot_status.c
    gui/setup.c
    gui/top_bar.c
    
    hal/eth.c
    hal/led.c
    hal/rtc.c
    hal/tft.c
    hal/touch.c
    hal/usart1.c
    
    network/arp.c
    network/ethernet.c
    network/icmp.c
    network/igmpv3.c
    network/inet.c
    network/ipv4.c
    network/sntp.c
    network/udp.c
    
    protobuf-c/messages_robocup_ssl_detection.pb-c.c
    protobuf-c/messages_robocup_ssl_geometry.pb-c.c
    protobuf-c/messages_robocup_ssl_geometry_legacy.pb-c.c
    protobuf-c/messages_robocup_ssl_refbox_log.pb-c.c
    protobuf-c/messages_robocup_ssl_wrapper.pb-c.c
    protobuf-c/messages_robocup_ssl_wrapper_legacy.pb-c.c
    protobuf-c/protobuf-c.c
    protobuf-c/radio_protocol_command.pb-c.c
    protobuf-c/radio_protocol_wrapper.pb-c.c
    
    util/wifi_module.c
    
    ${common_SOURCES}
    ${f7xx_SOURCES}
    ${robot_SOURCES}
    ${util_SOURCES}
    ${chibios_SOURCES}
    ${ugfx_SSD1963_SOURCES}
    ${gui_SOURCES}
)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    
    ${cmsis_core_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${f7xx_INCLUDE_DIRS}
    ${util_INCLUDE_DIRS}
    ${dsp_INCLUDE_DIRS}
    ${chibios_INCLUDE_DIRS}
    ${ugfx_SSD1963_INCLUDE_DIRS}
    ${gui_INCLUDE_DIRS}
)

add_compile_definitions(
    STM32F7XX
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
    -Wl,-Map=${RUN_EXEC}.map
)

# Bootloader code
add_executable(${BOOT_EXEC}
    ${bs2018_SOURCES}
)

target_compile_definitions(${BOOT_EXEC} PRIVATE
    ENV_BOOT
)

target_link_libraries(${BOOT_EXEC} c m)

target_link_options(${BOOT_EXEC} PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/boot.ld
    -Wl,-Map=${BOOT_EXEC}.map
)

include(../../cmake/post_build.cmake)
