cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        )

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/*.s
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        )

add_library(${PROJECT_NAME}-Libs OBJECT)

target_include_directories(${PROJECT_NAME}-Libs
        SYSTEM PUBLIC
        ${PROJECT_SOURCE_DIR}/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/protos_core
        ${CMAKE_CURRENT_SOURCE_DIR}/protos_can_device
        ${CMAKE_CURRENT_SOURCE_DIR}/sa_stm32_g4
        ${CMAKE_CURRENT_SOURCE_DIR}/1Wire
        )

target_compile_definitions(${PROJECT_NAME}-Libs
        PUBLIC
        )

target_sources(${PROJECT_NAME}-Libs
        PUBLIC
        ${HEADERS}
        PRIVATE
        ${SOURCES}
        )