# get version information
find_package(Git REQUIRED)
if(NOT GIT_FOUND)
    message(FATAL_ERROR "git not found!")
endif()

# describe git version
execute_process(COMMAND ${GIT_EXECUTABLE} describe --dirty
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                OUTPUT_VARIABLE GITSTR
                OUTPUT_STRIP_TRAILING_WHITESPACE)

# match git describe pattern
if(NOT "${GITSTR}" STREQUAL "")
    string(REGEX MATCH "[v]([0-9]*[.][0-9]*[.][0-9]*)([^ ]*)" GIT_DESC ${GITSTR})
endif()

if("${CMAKE_MATCH_1}" STREQUAL "")
    message(WARNING "No git version information")
    set(GIT_VERSION 0.0.0)
    set(GIT_VERSIONEXT "0.0.0-NA")
else()
    set(GIT_VERSION ${CMAKE_MATCH_1})
    set(GIT_VERSIONEXT ${CMAKE_MATCH_1}${CMAKE_MATCH_2})
    message(STATUS "GIT version ${GIT_VERSION}")
endif()

# try to break version number into pieces (major.minor.patch)
string(REPLACE "." ";" VERSION_LIST ${GIT_VERSION})
list(LENGTH VERSION_LIST VER_LEN)

if(${VER_LEN} GREATER 0)
    list(GET VERSION_LIST 0 GIT_VERSION_MAJOR)
else()
    set(GIT_VERSION_MAJOR 0)
endif()

if(${VER_LEN} GREATER 1)
    list(GET VERSION_LIST 1 GIT_VERSION_MINOR)
else()
    set(GIT_VERSION_MINOR 0)
endif()

if(${VER_LEN} GREATER 2)
    list(GET VERSION_LIST 2 GIT_VERSION_PATCH)
else()
    set(GIT_VERSION_PATCH 0)
endif()

# SHA1
execute_process(COMMAND ${GIT_EXECUTABLE} show -s "--format=%H"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                OUTPUT_VARIABLE GIT_SHA1
                OUTPUT_STRIP_TRAILING_WHITESPACE)
                
string(SUBSTRING ${GIT_SHA1} 0 8 GIT_SHA1_SHORT)
string(TOUPPER ${GIT_SHA1_SHORT} GIT_SHA1_SHORT)

message(STATUS "GIT SHA1 ${GIT_SHA1}")

# Commit date
execute_process(COMMAND ${GIT_EXECUTABLE} show -s "--format=%ci"
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                OUTPUT_VARIABLE GIT_COMMIT_DATE_ISO8601
                OUTPUT_STRIP_TRAILING_WHITESPACE)

message(STATUS "GIT commit date: ${GIT_COMMIT_DATE_ISO8601}")

# Local changes?
execute_process(COMMAND ${GIT_EXECUTABLE} status --porcelain -unormal
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                OUTPUT_VARIABLE GIT_STATUS
                OUTPUT_STRIP_TRAILING_WHITESPACE)

if(NOT "${GIT_STATUS}" STREQUAL "")
    set(GIT_IS_DIRTY "1")
else()
    set(GIT_IS_DIRTY "0")
endif()
        
message(STATUS "GIT dirty: ${GIT_IS_DIRTY}")
