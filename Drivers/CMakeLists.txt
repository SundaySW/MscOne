cmake_minimum_required(VERSION 3.16)

file(GLOB_RECURSE DRIVERS_HEADERS CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/*.hpp
        )

file(GLOB_RECURSE DRIVERS_SOURCES CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/*.s
        ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
        )

add_library(${PROJECT_NAME}-Drivers OBJECT)

target_include_directories(${PROJECT_NAME}-Drivers
        PUBLIC
        ${PROJECT_SOURCE_DIR}/Core/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Device/ST/${DEVICE_FAMILY}/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/CMSIS/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/${DEVICE_FAMILY}_HAL_Driver/Inc/Legacy
        ${CMAKE_CURRENT_SOURCE_DIR}/${DEVICE_FAMILY}_HAL_Driver/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/${DEVICE_FAMILY}_HAL_Driver/Src
        )

target_compile_definitions(${PROJECT_NAME}-Drivers
        PUBLIC
        ${DEVICE_NAME}
        )

target_sources(${PROJECT_NAME}-Drivers
        PUBLIC
        ${DRIVERS_HEADERS}
        PRIVATE
        ${DRIVERS_SOURCES}
        )