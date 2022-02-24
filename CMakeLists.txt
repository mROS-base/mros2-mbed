# Copyright (c) 2021 smorita_emb
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)

set(MBED_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os
    CACHE INTERNAL "")
set(MBED_CONFIG_PATH
    ${CMAKE_CURRENT_BINARY_DIR}
    CACHE INTERNAL "")
set(APP_TARGET mros2-mbed)

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH})

if(DEFINED ENV{APPNAME})
  set(APP_NAME $ENV{APPNAME})
else()
  set(APP_NAME "echoreply_string")
endif()
message(STATUS "APP_NAME=${APP_NAME}")
add_executable(${APP_TARGET} workspace/${APP_NAME}/app.cpp)

target_link_libraries(${APP_TARGET} mbed-os mbed-lwipstack mros2)

execute_process(COMMAND git submodule update --init --recursive
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mros2)
add_library(mros2 INTERFACE)
add_subdirectory(mros2)

set(RTPS_CONFIG_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")

target_include_directories(
  mros2-mbed
  PRIVATE ${RTPS_CONFIG_INCLUDE_DIR}
)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
  set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
