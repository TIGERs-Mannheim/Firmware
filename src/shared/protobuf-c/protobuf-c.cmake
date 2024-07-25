list(APPEND protobuf_c_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/messages_robocup_ssl_detection.pb-c.c
    ${CMAKE_CURRENT_LIST_DIR}/messages_robocup_ssl_geometry.pb-c.c
    ${CMAKE_CURRENT_LIST_DIR}/messages_robocup_ssl_geometry_legacy.pb-c.c
    ${CMAKE_CURRENT_LIST_DIR}/messages_robocup_ssl_refbox_log.pb-c.c
    ${CMAKE_CURRENT_LIST_DIR}/messages_robocup_ssl_wrapper.pb-c.c
    ${CMAKE_CURRENT_LIST_DIR}/messages_robocup_ssl_wrapper_legacy.pb-c.c
    ${CMAKE_CURRENT_LIST_DIR}/protobuf-c.c
    ${CMAKE_CURRENT_LIST_DIR}/radio_protocol_command.pb-c.c
    ${CMAKE_CURRENT_LIST_DIR}/radio_protocol_wrapper.pb-c.c
)

list(APPEND protobuf_c_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/..)
