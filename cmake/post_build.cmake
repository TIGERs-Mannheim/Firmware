if(TARGET ${RUN_EXEC})
    add_custom_command(TARGET ${RUN_EXEC}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary ${RUN_EXEC} ${PROJECT_NAME}.bin)

    if(GENERATE_LISTING)
        add_custom_command(TARGET ${RUN_EXEC}
            POST_BUILD
            COMMAND ${CMAKE_OBJDUMP} -h -t -S -C -D -l ${RUN_EXEC} > ${RUN_EXEC}.lss)
    endif()
    
    list(APPEND binaries ${PROJECT_NAME}/${RUN_EXEC})
endif()

if(TARGET ${BOOT_EXEC})
    add_custom_command(TARGET ${BOOT_EXEC}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary ${BOOT_EXEC} ${PROJECT_NAME}_boot.bin)

    if(GENERATE_LISTING)
        add_custom_command(TARGET ${BOOT_EXEC}
            POST_BUILD
            COMMAND ${CMAKE_OBJDUMP} -h -t -S -C -D -l ${BOOT_EXEC} > ${BOOT_EXEC}.lss)
    endif()
    
    list(APPEND binaries ${PROJECT_NAME}/${BOOT_EXEC})
endif()

set(APPLICATIONS ${APPLICATIONS} ${binaries} PARENT_SCOPE)
set(${PROJECT_NAME}_EXECS ${binaries} PARENT_SCOPE)
