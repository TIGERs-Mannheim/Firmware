cmake_minimum_required(VERSION 3.16)

project(main2019)

set(RUN_EXEC "${PROJECT_NAME}.elf")
set(BOOT_EXEC "${PROJECT_NAME}_boot.elf")

include(../../cmake/options.cmake)
include(../libs/libs.cmake)
include(../robot/robot.cmake)
include(../shared/shared.cmake)

list(APPEND main2019_SOURCES
    main.c
    network_impl.c
    presenter.c
    robot_impl.c
    tiger_bot.c
    shell_commands.c
    system.c
    
    misc/diskio.c
    misc/inventory.c
    misc/variant.c
    
    test/test_common.c
    test/test_imu.c
    test/test_kicker.c
    test/test_motor.c
    test/test_experimental.c
    test/tests.c
    
    gui/ball_view.c
    gui/dribbler.c
    gui/imu.c
    gui/ir_gui.c
    gui/versions.c
    
    sys/i2c2.c
    sys/i2c5.c
    sys/sdcard.c
    sys/spi1.c
    sys/spi2.c
    sys/spi4.c
    sys/tim2.c
    sys/tim5.c
    sys/tim13.c
    sys/tim14.c
    sys/usart1.c
    sys/usart2.c
    sys/usart3.c
    sys/uart4.c
    sys/uart5.c
    sys/usart6.c
    sys/uart7.c
    sys/uart8.c
    sys/usb_fs.c
    
    dev/adc_kicker.c
    dev/buzzer.c
    dev/drib_ir.c
    dev/imu.c
    dev/leds_front.c
    dev/leds_main.c
    dev/mag.c
    dev/motors.c
    dev/port_exp.c
    dev/power_mon.c
    dev/raspberry_pi.c
    dev/shell.c
    dev/sky_fem.c
    dev/sx1280.c
    dev/tft.c
    dev/touch.c
    
    ${shared_SOURCES}
    ${cmsis_core_SOURCES}
    ${h7xx_SOURCES}
    ${util_SOURCES}
    ${fatfs_SOURCES}
    ${dsp_SOURCES}
    ${chibios_SOURCES}
    ${robot_SOURCES}
    ${math_SOURCES}
    ${ugfx_ILI9341_SOURCES}
    ${gui_SOURCES}
)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}
    
    ${cmsis_core_INCLUDE_DIRS}
    ${shared_INCLUDE_DIRS}
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
    STM32H743xx
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
    -u_printf_float
    -u_scanf_float
    -T${CMAKE_CURRENT_SOURCE_DIR}/run.ld
    -L${CMAKE_CURRENT_SOURCE_DIR}
    -L${CMAKE_CURRENT_SOURCE_DIR}/../libs/chibios
    -Wl,-Map=${RUN_EXEC}.map
)

# Embed motor and dribbler controller code into normal application code
add_custom_command(OUTPUT motor2019.bin
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:motor2019.elf> motor2019.bin
    DEPENDS motor2019.elf)

EmbedResource(${RUN_EXEC} motor2019.bin ${CMAKE_CURRENT_SOURCE_DIR}/generated/mcu_motor_code.c)

add_custom_command(OUTPUT ir2019.bin
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:ir2019.elf> ir2019.bin
    DEPENDS ir2019.elf)

EmbedResource(${RUN_EXEC} ir2019.bin ${CMAKE_CURRENT_SOURCE_DIR}/generated/mcu_dribbler_code.c)

# Bootloader code
add_executable(${BOOT_EXEC}
    main_bootloader.c
    system.c
    
	sys/tim2.c
	sys/usart2.c
	dev/leds_main.c
	dev/leds_front.c
    
    ${chibios_SOURCES}
    ${util_SOURCES}
    ${h7xx_SOURCES}
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
