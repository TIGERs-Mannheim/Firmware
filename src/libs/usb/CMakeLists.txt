list(APPEND usb_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/usb_hcd.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_msc.c
    ${CMAKE_CURRENT_LIST_DIR}/usb.c
)

list(APPEND usb_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})
