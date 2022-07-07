cmake_minimum_required(VERSION 3.16)

set(CMAKE_TOOLCHAIN_FILE cmake/arm-none-eabi.toolchain.cmake)
set(CMAKE_VERBOSE_MAKEFILE OFF)

include(cmake/git_version.cmake)

project(Firmware LANGUAGES C CXX ASM VERSION ${GIT_VERSION})

# store GIT version info to file
configure_file(src/libs/common/git_version.h.in ${CMAKE_CURRENT_SOURCE_DIR}/src/libs/common/git_version.h)

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
        " OpenOCD not found. You will not be able to use flash targets but you can still compile the code.\n \n"
        " Plase specify a path to your OpenOCD installation (top-level folder) with cmake argument:\n"
        "   -DCMAKE_PREFIX_PATH=<path to OpenOCD>\n \n"
        " (You can specify multiple paths with a semicolon-separated list)")
endif()

# Build options
option(BUILD_MAIN2019 "Build binary for v2020 main microcontroller" ON)
option(BUILD_MOTOR2019 "Build binary for v2020 motor microcontroller" ON)
option(BUILD_IR2019 "Build binary for v2020 IR microcontroller" ON)
option(BUILD_MAIN2016 "Build binary for v2016 main microcontroller" ON)
option(BUILD_BS2018 "Build binary for v2018 base station microcontroller" ON)

option(GENERATE_LISTING "Generate disassembly" OFF)

# Add subprojects based on selected options
if(BUILD_MAIN2019)
    add_subdirectory(src/main2019)
endif()

if(BUILD_MOTOR2019)
    add_subdirectory(src/motor2019)
endif()

if(BUILD_IR2019)
    add_subdirectory(src/ir2019)
endif()

if(BUILD_MAIN2016)	
    add_subdirectory(src/main2016)
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
        DEPENDS ${main2019_EXECS} ${motor2019_EXECS} ${ir2019_EXECS}
    )
    
    add_custom_target(flash-mb2016
        COMMAND ${OPENOCD} -c "set folder ${CMAKE_BINARY_DIR}" -f openocd/flash-mb2016.cfg
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${main2016_EXECS}
    )
    
    add_custom_target(flash-bs2018
        COMMAND ${OPENOCD} -c "set folder ${CMAKE_BINARY_DIR}" -f openocd/flash-bs2018.cfg
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        DEPENDS ${bs2018_EXECS}
    )
endif()
