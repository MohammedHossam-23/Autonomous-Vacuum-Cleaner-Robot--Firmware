cmake_minimum_required(VERSION 3.20)

project(avcr_low_level_control LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# Cross builds (toolchain file) produce the firmware; host builds produce the
# reusable libraries, the CLI and the unit tests.
if(CMAKE_CROSSCOMPILING)
    add_subdirectory(firmware)
else()
    enable_testing()
    add_subdirectory(host)
endif()
