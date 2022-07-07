list(APPEND gui_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/bot_cover.c
    ${CMAKE_CURRENT_LIST_DIR}/fw_update.c
    ${CMAKE_CURRENT_LIST_DIR}/icons.c
    ${CMAKE_CURRENT_LIST_DIR}/logging.c
    ${CMAKE_CURRENT_LIST_DIR}/main_status.c
    ${CMAKE_CURRENT_LIST_DIR}/menu.c
    ${CMAKE_CURRENT_LIST_DIR}/select_botid.c
    ${CMAKE_CURRENT_LIST_DIR}/test_drive.c
    ${CMAKE_CURRENT_LIST_DIR}/test_kicker.c
    ${CMAKE_CURRENT_LIST_DIR}/top_bar.c
    ${CMAKE_CURRENT_LIST_DIR}/wifi.c
)

list(APPEND gui_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/..)
