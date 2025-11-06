cmake_minimum_required(VERSION 3.22)
project(motor VERSION 0.0.1 LANGUAGES C CXX ASM)

file(GLOB_RECURSE SOURCES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc"
)

add_library(motor STATIC ${SOURCES})
target_include_directories(motor
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/internal
)

#target_compile_options(motor PRIVATE -O3 -Wall -Werror)

target_link_libraries(motor PUBLIC bsp m)
target_link_libraries(motor PUBLIC components::utils)

set_target_properties(motor PROPERTIES
        C_STANDARD 17
        CXX_STANDARD 23
        CXX_STANDARD_REQUIRED YES
        POSITION_INDEPENDENT_CODE NO
)

add_library(components::motor ALIAS motor)