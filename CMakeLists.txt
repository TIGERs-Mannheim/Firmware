cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE cmake/arm-none-eabi.toolchain.cmake)
set(CMAKE_VERBOSE_MAKEFILE OFF)

include(cmake/git_version.cmake)
include(cmake/EmbedResource.cmake)

project(Firmware LANGUAGES C CXX ASM VERSION ${GIT_VERSION})

# store GIT version info to file
configure_file(src/shared/git_version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/shared/git_version.h)

# Check compiler and build type
if(NOT ${CMAKE_C_COMPILER} MATCHES ${TOOLCHAIN_PREFIX})
    message(FATAL_ERROR 
        " Incompatible compiler found!\n"
        " ASM Compiler: ${CMAKE_ASM_COMPILER}\n"
        " C   Compiler: ${CMAKE_C_COMPILER}\n"
        " C++ Compiler: ${CMAKE_CXX_COMPILER}\n \n"
        " A compiler containing the name '${TOOLCHAIN_PREFIX}' is required.\n"
        " Delete your build folder (${CMAKE_BINARY_DIR}) and\n"
        " specify a path to a compatible compiler with cmake argument:\n"
        "   -DCMAKE_PREFIX_PATH=<path to compiler>\n")
endif()

set(default_build_type "Release")

if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
        STRING "Choose the type of build." FORCE)
endif()

find_program(OPENOCD "openocd")

if(NOT OPENOCD)
    message(WARNING 
        " OpenOCD not found. You will not be able to use flash/debug targets but you can still compile the code.\n \n"
        " Please specify a path to your OpenOCD installation (top-level folder) with cmake argument:\n"
        "   -DCMAKE_PREFIX_PATH=<path to OpenOCD>\n \n"
        " (You can specify multiple paths with a semicolon-separated list)")
else()
    message(STATUS "OpenOCD found at: ${OPENOCD}")
endif()

find_program(TIGERFLASH "tigerflash")

if(NOT TIGERFLASH)
    message(WARNING 
        " tigerflash not found. You will not be able to use flash targets but you can still compile the code.\n \n"
        " Please specify a path to your tigerflash installation (top-level folder) with cmake argument:\n"
        "   -DCMAKE_PREFIX_PATH=<path to tigerflash>\n \n"
        " (You can specify multiple paths with a semicolon-separated list)")
else()
    message(STATUS "tigerflash found at: ${TIGERFLASH}")
endif()

# Build options
option(BUILD_MAIN2019 "Build binary for v2020 main microcontroller" ON)
option(BUILD_MOTOR2019 "Build binary for v2020 motor microcontroller" ON)
option(BUILD_IR2019 "Build binary for v2020 IR microcontroller" ON)
option(BUILD_BS2018 "Build binary for v2018 base station microcontroller" ON)
option(AUTO_FLASH "Automatically flash executable to target if tigerflash is available" OFF)

option(GENERATE_LISTING "Generate disassembly" OFF)

# Add subprojects based on selected options
if(BUILD_MOTOR2019)
    add_subdirectory(src/motor2019)
endif()

if(BUILD_IR2019)
    add_subdirectory(src/ir2019)
endif()

if(BUILD_MAIN2019)
    add_subdirectory(src/main2019)
endif()

if(BUILD_BS2018)
    add_subdirectory(src/bs2018)
endif()

# Print size information for all built targets
list(TRANSFORM APPLICATIONS PREPEND src/)

add_custom_target(size_info ALL
    COMMAND ${TOOL_SIZE_UTIL} -d -A ${APPLICATIONS} && ${TOOL_SIZE_UTIL} -d ${APPLICATIONS}
    DEPENDS ${APPLICATIONS}
)

# Convenience targets for flashing from command line
if(OPENOCD)
    add_custom_target(flash-mb2019
        COMMAND ${OPENOCD} -c "set folder ${CMAKE_BINARY_DIR}" -f openocd/flash-mb2019.cfg
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${main2019_EXECS}
    )
    
    add_custom_target(flash-bs2018
        COMMAND ${OPENOCD} -c "set folder ${CMAKE_BINARY_DIR}" -f openocd/flash-bs2018.cfg
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${bs2018_EXECS}
    )
endif()

if(TIGERFLASH)
    if(AUTO_FLASH AND BUILD_MAIN2019 AND NOT BUILD_BS2018)
        add_custom_target(tigerflash-auto-mb2019 ALL
            COMMAND ${TIGERFLASH} -a 0x08100000 -s 768 -d TigerBotV2020 -c -q -w ${CMAKE_BINARY_DIR}/src/main2019/main2019.bin
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            DEPENDS ${main2019_EXECS} size_info
        )
    endif()

    add_custom_target(tigerflash-mb2019
        COMMAND ${TIGERFLASH} -a 0x08100000 -s 768 -d TigerBotV2020 -c -q -w ${CMAKE_BINARY_DIR}/src/main2019/main2019.bin
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${main2019_EXECS} size_info
    )

    if(AUTO_FLASH AND BUILD_BS2018 AND NOT BUILD_MAIN2019)
        add_custom_target(tigerflash-auto-bs2018 ALL
            COMMAND ${TIGERFLASH} -a 0x08020000 -s 384 -d BSV2018 -c -q -w ${CMAKE_BINARY_DIR}/src/bs2018/bs2018.bin
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            DEPENDS ${bs2018_EXECS} size_info
        )
    endif()

    add_custom_target(tigerflash-bs2018
        COMMAND ${TIGERFLASH} -a 0x08020000 -s 384 -d BSV2018 -c -q -w ${CMAKE_BINARY_DIR}/src/bs2018/bs2018.bin
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${bs2018_EXECS} size_info
    )
endif()

include(ExternalProject)
ExternalProject_Add(tests
    PREFIX tests
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/tests
    BINARY_DIR ${CMAKE_BINARY_DIR}/tests/build
    BUILD_ALWAYS 1
    EXCLUDE_FROM_ALL 1
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
)
