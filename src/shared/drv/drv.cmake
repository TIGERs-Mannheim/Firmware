list(APPEND drv_common_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/adc_kicker.c
    ${CMAKE_CURRENT_LIST_DIR}/buzzer.c
    ${CMAKE_CURRENT_LIST_DIR}/fem_interface.c
    ${CMAKE_CURRENT_LIST_DIR}/imu_icm20689.c
    ${CMAKE_CURRENT_LIST_DIR}/io_tca9539.c
    ${CMAKE_CURRENT_LIST_DIR}/led_mono.c
    ${CMAKE_CURRENT_LIST_DIR}/led_rgbw.c
    ${CMAKE_CURRENT_LIST_DIR}/mag_lis3.c
    ${CMAKE_CURRENT_LIST_DIR}/mcu_dribbler.c
    ${CMAKE_CURRENT_LIST_DIR}/mcu_motor.c
    ${CMAKE_CURRENT_LIST_DIR}/mpu_ext.c
    ${CMAKE_CURRENT_LIST_DIR}/pwr_ina226.c
    ${CMAKE_CURRENT_LIST_DIR}/sky66112.c
    ${CMAKE_CURRENT_LIST_DIR}/sx1280_lld.c
#    ${CMAKE_CURRENT_LIST_DIR}/sx1280_os.c
    ${CMAKE_CURRENT_LIST_DIR}/sx1280.c
    ${CMAKE_CURRENT_LIST_DIR}/tca9548a.c
    ${CMAKE_CURRENT_LIST_DIR}/tcs3472.c
    ${CMAKE_CURRENT_LIST_DIR}/touch_ad7843.c
)

list(APPEND drv_common_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}
)

list(APPEND f7xx_SOURCES
    ${drv_common_SOURCES}
)

list(APPEND f7xx_INCLUDE_DIRS
    ${drv_common_INCLUDE_DIRS}
)

list(APPEND h7xx_SOURCES
    ${drv_common_SOURCES}
)

list(APPEND h7xx_INCLUDE_DIRS
    ${drv_common_INCLUDE_DIRS}
)
