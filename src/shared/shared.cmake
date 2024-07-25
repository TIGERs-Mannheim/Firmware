include(${CMAKE_CURRENT_LIST_DIR}/drv/drv.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/gui/gui.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/hal/hal.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/math/math.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/module/module.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/net/net.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/protobuf-c/protobuf-c.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/util/util.cmake)

list(APPEND shared_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/log_msgs.c
    ${CMAKE_CURRENT_LIST_DIR}/version.c
)

list(APPEND shared_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})
