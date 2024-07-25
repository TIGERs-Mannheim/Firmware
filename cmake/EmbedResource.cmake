# Based on: https://jonathanhamberg.com/post/cmake-file-embedding/

set(EMBED_RESOURCE_FILE "${CMAKE_CURRENT_LIST_DIR}/EmbedResource.cmake")

function(EmbedResource THE_TARGET BIN_FILE C_FILE)
    target_sources(${THE_TARGET} PUBLIC ${C_FILE})
    
    add_custom_command(
        OUTPUT ${C_FILE}
        COMMAND ${CMAKE_COMMAND}
        -DBIN_FILE=${BIN_FILE}
        -DC_FILE=${C_FILE}
        -P ${EMBED_RESOURCE_FILE}
        DEPENDS ${BIN_FILE}
    )
endfunction()

function(GenerateResource BIN_FILE C_FILE)
    file(READ ${BIN_FILE} HEX_CONTENT HEX)
    
    string(REGEX MATCHALL "([A-Fa-f0-9][A-Fa-f0-9])" SEPARATED_HEX ${HEX_CONTENT})
    
    # Create a counter so that we only have 16 hex bytes per line
    set(counter 0)
    
    # Iterate through each of the bytes from the source file
    foreach(hex IN LISTS SEPARATED_HEX)
    	# Write the hex string to the line with an 0x prefix
    	# and a , postfix to seperate the bytes of the file.
        string(APPEND output_c "0x${hex},")
        
        # Increment the element counter before the newline.
        math(EXPR counter "${counter}+1")
        if(counter GREATER 15)
        	# Write a newline so that all of the array initializer
        	# gets spread across multiple lines.
            string(APPEND output_c "\n    ")
            set(counter 0)
        endif()
    endforeach()
    
    get_filename_component(c_name ${C_FILE} NAME_WE)
    
    # Generate the contents that will be contained in the source file.
    set(output_c "#include \"${c_name}.h\"

const uint8_t ${c_name}_data[] = {
    ${output_c}
}\;

const uint32_t ${c_name}_size = sizeof(${c_name}_data)\;
")

    # Generate the contents that will be contained in the header file.
    set(output_h "#pragma once

#include <stdint.h>

extern const uint8_t ${c_name}_data[]\;
extern const uint32_t ${c_name}_size\;
")
        
    get_filename_component(OUT_DIR ${C_FILE} DIRECTORY)
    
    file(WRITE ${OUT_DIR}/${c_name}.h ${output_h})
    file(WRITE ${OUT_DIR}/${c_name}.c ${output_c})
    
    message(STATUS "Generated: ${OUT_DIR}/${c_name}.c from ${BIN_FILE}")
endfunction()

# This is used if the file is called as a script
if(DEFINED BIN_FILE AND DEFINED C_FILE)
    GenerateResource(${BIN_FILE} ${C_FILE})
endif()
