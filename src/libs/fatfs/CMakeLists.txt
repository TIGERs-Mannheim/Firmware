list(APPEND fatfs_SOURCES
	${CMAKE_CURRENT_LIST_DIR}/ff.c
	${CMAKE_CURRENT_LIST_DIR}/option/ccsbcs.c
	${CMAKE_CURRENT_LIST_DIR}/option/syscall.c
)

list(APPEND fatfs_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR})
