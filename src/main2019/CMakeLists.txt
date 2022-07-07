cmake_minimum_required(VERSION 3.16)

project(main2019)

set(RUN_EXEC "${PROJECT_NAME}.elf")
set(BOOT_EXEC "${PROJECT_NAME}_boot.elf")

include(../../cmake/options.cmake)
include(../libs/CMakeLists.txt)

list(APPEND main2019_SOURCES
    cli.c
    ext.c
    inventory.c
    ir.c
    kicker.c
    main.c
    microphone.c
    motors.c
    network_impl.c
    pattern_ident.c
    power.c
    presenter.c
    robot_impl.c
    robot_pi.c
    sdcard.c
    spi2.c
    spi4.c
    
    test/test.c
    test/test_imu.c
    test/test_kicker.c
    test/test_motor.c
    test/test_experimental.c
    
    gui/ball_view.c
    gui/dribbler.c
    gui/imu.c
    gui/ir_gui.c
    gui/versions.c
    
    hal/buzzer.c
    hal/diskio.c
    hal/i2c2.c
    hal/led.c
    hal/port_ex.c
    hal/tft.c
    hal/usart2.c
    hal/usb_fs.c
    
    ${common_SOURCES}
    ${cmsis_core_SOURCES}
    ${h7xx_SOURCES}
    ${util_SOURCES}
    ${fatfs_SOURCES}
    ${dsp_SOURCES}
    ${chibios_SOURCES}
    ${robot_SOURCES}
    ${usb_SOURCES}
    ${ugfx_ILI9341_SOURCES}
    ${gui_SOURCES}
)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    
    ${cmsis_core_INCLUDE_DIRS}
    ${common_INCLUDE_DIRS}
    ${h7xx_INCLUDE_DIRS}
    ${util_INCLUDE_DIRS}
    ${fatfs_INCLUDE_DIRS}
    ${dsp_INCLUDE_DIRS}
    ${chibios_INCLUDE_DIRS}
    ${robot_INCLUDE_DIRS}
    ${usb_INCLUDE_DIRS}
    ${ugfx_ILI9341_INCLUDE_DIRS}
    ${gui_INCLUDE_DIRS}
)

add_compile_definitions(
    STM32H7XX
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
    ${main2019_SOURCES}
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
    ${main2019_SOURCES}
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
