cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

include($ENV{PICO_EXTRAS_PATH}/external/pico_extras_import.cmake)

project(amb1805 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

add_compile_options(-Wall
        -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
        -Wno-unused-function # we have some for the docs that aren't called
        )
if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-Wno-maybe-uninitialized)
endif()

add_executable(amb1805
        main.c am1805.c test_am1805.c
        )

target_include_directories(amb1805 PRIVATE $ENV{PICO_EXTRAS_PATH})

# pull in common dependencies and additional i2c hardware support
target_link_libraries(amb1805 pico_stdlib hardware_i2c hardware_sleep hardware_rosc hardware_rtc pico_util)

# create map/bin/hex file etc.
pico_add_extra_outputs(amb1805)

pico_enable_stdio_usb(amb1805 1)
pico_enable_stdio_uart(amb1805 1)
