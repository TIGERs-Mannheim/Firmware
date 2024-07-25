list(APPEND module_common_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/bootloader.c
    ${CMAKE_CURRENT_LIST_DIR}/radio/radio_phy.c
    ${CMAKE_CURRENT_LIST_DIR}/radio/radio_buf.c
    ${CMAKE_CURRENT_LIST_DIR}/radio/radio_module.c
    ${CMAKE_CURRENT_LIST_DIR}/radio/radio_base.c
    ${CMAKE_CURRENT_LIST_DIR}/radio/radio_bot.c
    ${CMAKE_CURRENT_LIST_DIR}/radio/radio_settings.c
)

list(APPEND module_common_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/..
)

list(APPEND h7xx_SOURCES
    ${module_common_SOURCES}
    ${CMAKE_CURRENT_LIST_DIR}/kicker.c
    ${CMAKE_CURRENT_LIST_DIR}/pattern_ident.c
    ${CMAKE_CURRENT_LIST_DIR}/power_control.c
    ${CMAKE_CURRENT_LIST_DIR}/robot_pi.c
)

list(APPEND h7xx_INCLUDE_DIRS
    ${module_common_INCLUDE_DIRS}
)

list(APPEND f7xx_SOURCES
    ${module_common_SOURCES}
)

list(APPEND f7xx_INCLUDE_DIRS
    ${module_common_INCLUDE_DIRS}
)
