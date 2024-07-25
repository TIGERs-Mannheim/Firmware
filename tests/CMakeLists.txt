cmake_minimum_required(VERSION 3.16)
project(test_libs)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(${CMAKE_CURRENT_LIST_DIR}/../src/shared/shared.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/libs/CMakeLists.txt)

enable_testing()
list(APPEND test_libs_SOURCES
    ${util_TEST_SOURCES}
    ${math_TEST_SOURCES}
)

add_executable(${PROJECT_NAME}
    ${test_libs_SOURCES}
    arm_math_mock/arm_math.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${math_INCLUDE_DIRS}
    arm_math_mock
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME})
install(FILES ${CMAKE_BINARY_DIR}/CTestTestfile.cmake DESTINATION bin)

