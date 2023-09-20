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
  set(APP_NAME "echoback_string")
endif()
message(STATUS "APP_NAME=${APP_NAME}")

# specify your own file(s) for the application
set(APP_SRCS
  workspace/${APP_NAME}/app.cpp
)
message(STATUS "APP_SRCS=${APP_SRCS}")

add_executable(${APP_TARGET}
  platform/mros2-platform.cpp
  ${APP_SRCS}
)

target_link_libraries(${APP_TARGET} mbed-os mbed-lwipstack mros2)

execute_process(COMMAND git submodule update --init --recursive
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/mros2)
add_library(mros2 INTERFACE)
add_subdirectory(mros2)

# Generate plaform/templates.hpp from mros2 application code
## You can add dir(s) in `--indir` if you have several dirs 
## that contain application code files
execute_process(COMMAND
  python3 ${CMAKE_CURRENT_SOURCE_DIR}/mros2/mros2_header_generator/templates_generator.py --outdir ${CMAKE_CURRENT_SOURCE_DIR}/platform --indir ${CMAKE_CURRENT_SOURCE_DIR}/workspace/${APP_NAME}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

set(MROS2_TARGET_DIR "${PROJECT_SOURCE_DIR}/platform")

target_include_directories(
  mros2-mbed
  PRIVATE workspace/${APP_NAME}
  PRIVATE ${MROS2_TARGET_DIR}
)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
  set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
