cmake_minimum_required(VERSION 3.12)

set(PICO_SDK_FETCH_FROM_GIT on)
# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(co2control-fw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)


# Initialize the SDK
pico_sdk_init()


add_subdirectory(src)

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        -Wno-maybe-uninitialized
        )
