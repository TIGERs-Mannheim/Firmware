list(APPEND chibios_SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/port/chcoreasm_v7m.s
    ${CMAKE_CURRENT_LIST_DIR}/port/crt0_v7m.s
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chcond.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chdebug.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chdynamic.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chevents.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chheap.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chmboxes.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chmemcore.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chmempools.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chmsg.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chmtx.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chqueues.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chregistry.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chschd.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chsem.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chstats.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chsys.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chthreads.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chtm.c
    ${CMAKE_CURRENT_LIST_DIR}/kernel/chvt.c
    ${CMAKE_CURRENT_LIST_DIR}/port/chcore.c
    ${CMAKE_CURRENT_LIST_DIR}/port/chcore_v7m.c
    ${CMAKE_CURRENT_LIST_DIR}/port/cpu_load.c
    ${CMAKE_CURRENT_LIST_DIR}/port/crt1.c
    ${CMAKE_CURRENT_LIST_DIR}/port/fault_handler.c
    ${CMAKE_CURRENT_LIST_DIR}/port/late_init.c
    ${CMAKE_CURRENT_LIST_DIR}/port/syscalls.c
    ${CMAKE_CURRENT_LIST_DIR}/port/vectors.c
)

list(APPEND chibios_INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/kernel
    ${CMAKE_CURRENT_LIST_DIR}/port
)
