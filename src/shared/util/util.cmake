list(APPEND util_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/bmp.c
    ${CMAKE_CURRENT_LIST_DIR}/bootloader_protocol.c
    ${CMAKE_CURRENT_LIST_DIR}/cobs.c
    ${CMAKE_CURRENT_LIST_DIR}/config.c
    ${CMAKE_CURRENT_LIST_DIR}/cpu_load.c
    ${CMAKE_CURRENT_LIST_DIR}/crc.c
    ${CMAKE_CURRENT_LIST_DIR}/crc8.c
    ${CMAKE_CURRENT_LIST_DIR}/device_profiler.c
    ${CMAKE_CURRENT_LIST_DIR}/fault_handler.c
    ${CMAKE_CURRENT_LIST_DIR}/fifo_lin.c
    ${CMAKE_CURRENT_LIST_DIR}/fifo.c
    ${CMAKE_CURRENT_LIST_DIR}/flash_fs.c
    ${CMAKE_CURRENT_LIST_DIR}/fw_loader_fatfs.c
    ${CMAKE_CURRENT_LIST_DIR}/fw_updater.c
    ${CMAKE_CURRENT_LIST_DIR}/list.c
    ${CMAKE_CURRENT_LIST_DIR}/log.c
    ${CMAKE_CURRENT_LIST_DIR}/log_file.c
    ${CMAKE_CURRENT_LIST_DIR}/network_print.c
    ${CMAKE_CURRENT_LIST_DIR}/queue.c
    ${CMAKE_CURRENT_LIST_DIR}/rf_queue.c
    ${CMAKE_CURRENT_LIST_DIR}/shell_cmd.c
    ${CMAKE_CURRENT_LIST_DIR}/shell.c
    ${CMAKE_CURRENT_LIST_DIR}/songs.c
    ${CMAKE_CURRENT_LIST_DIR}/ssl_vision.c
    ${CMAKE_CURRENT_LIST_DIR}/st_bootloader.c
    ${CMAKE_CURRENT_LIST_DIR}/test.c
)

list(APPEND util_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/..)
