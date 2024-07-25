list(APPEND hal_common_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/gpio_pin_st.c
    ${CMAKE_CURRENT_LIST_DIR}/i2c_lld_soft.c
    ${CMAKE_CURRENT_LIST_DIR}/i2c.c
    ${CMAKE_CURRENT_LIST_DIR}/init_hal.c
    ${CMAKE_CURRENT_LIST_DIR}/rng.c
    ${CMAKE_CURRENT_LIST_DIR}/spi_lld.c
    ${CMAKE_CURRENT_LIST_DIR}/spi.c
    ${CMAKE_CURRENT_LIST_DIR}/sys_time.c
    ${CMAKE_CURRENT_LIST_DIR}/timer_simple_lld.c
    ${CMAKE_CURRENT_LIST_DIR}/timer_trigger_lld.c
)

list(APPEND hal_common_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/..
)

list(APPEND usb_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/usb/usb_hcd.c
    ${CMAKE_CURRENT_LIST_DIR}/usb/usb_msc.c
    ${CMAKE_CURRENT_LIST_DIR}/usb/usb.c
)

list(APPEND f0xx_SOURCES
	${CMAKE_CURRENT_LIST_DIR}/f0xx/startup_stm32f031x6.s
    ${CMAKE_CURRENT_LIST_DIR}/f0xx/hal/system_init.c
)

list(APPEND f0xx_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/f0xx
)

list(APPEND f7xx_SOURCES
    ${hal_common_SOURCES}
    ${CMAKE_CURRENT_LIST_DIR}/f7xx/hal/eth.c
    ${CMAKE_CURRENT_LIST_DIR}/f7xx/hal/flash.c
    ${CMAKE_CURRENT_LIST_DIR}/f7xx/hal/uart_dma.c
)

list(APPEND f7xx_INCLUDE_DIRS
    ${hal_common_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/f7xx
)

list(APPEND h7xx_SOURCES
    ${hal_common_SOURCES}
    ${usb_SOURCES}
    ${CMAKE_CURRENT_LIST_DIR}/h7xx/hal/flash.c
    ${CMAKE_CURRENT_LIST_DIR}/h7xx/hal/i2c_lld_hw.c
    ${CMAKE_CURRENT_LIST_DIR}/h7xx/hal/uart_dma.c
    ${CMAKE_CURRENT_LIST_DIR}/h7xx/hal/uart_fifo.c
    ${CMAKE_CURRENT_LIST_DIR}/h7xx/hal/timer_opm.c
)

list(APPEND h7xx_INCLUDE_DIRS
    ${hal_common_INCLUDE_DIRS}
    ${CMAKE_CURRENT_LIST_DIR}/h7xx
)
