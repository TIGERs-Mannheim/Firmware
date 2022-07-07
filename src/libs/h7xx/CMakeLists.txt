list(APPEND h7xx_SOURCES
	${CMAKE_CURRENT_LIST_DIR}/util/flash.c
	${CMAKE_CURRENT_LIST_DIR}/util/i2c.c
	${CMAKE_CURRENT_LIST_DIR}/util/st_bootloader.c
	${CMAKE_CURRENT_LIST_DIR}/util/uart_fifo.c
)

list(APPEND h7xx_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})
