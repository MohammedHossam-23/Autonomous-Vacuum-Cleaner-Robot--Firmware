# Cross-compilation toolchain for the STM32F411 firmware target.
#
# The ARM GCC bundled with STM32CubeIDE is reused when present; override with
# -DARM_TOOLCHAIN_BIN=/path/to/bin or the ARM_TOOLCHAIN_BIN environment variable.

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(DEFINED ENV{ARM_TOOLCHAIN_BIN})
    set(_avcr_toolchain_bin "$ENV{ARM_TOOLCHAIN_BIN}")
elseif(DEFINED ARM_TOOLCHAIN_BIN)
    set(_avcr_toolchain_bin "${ARM_TOOLCHAIN_BIN}")
else()
    file(GLOB _avcr_gcc_candidates
        "/opt/st/stm32cubeide_*/plugins/*gnu-tools-for-stm32*/tools/bin/arm-none-eabi-gcc")
    if(_avcr_gcc_candidates)
        list(GET _avcr_gcc_candidates 0 _avcr_gcc)
        get_filename_component(_avcr_toolchain_bin "${_avcr_gcc}" DIRECTORY)
    endif()
endif()

if(NOT _avcr_toolchain_bin)
    message(FATAL_ERROR
        "arm-none-eabi-gcc not found. Set ARM_TOOLCHAIN_BIN to the toolchain bin directory.")
endif()

set(CMAKE_C_COMPILER   "${_avcr_toolchain_bin}/arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "${_avcr_toolchain_bin}/arm-none-eabi-gcc")
set(CMAKE_OBJCOPY      "${_avcr_toolchain_bin}/arm-none-eabi-objcopy")
set(CMAKE_SIZE         "${_avcr_toolchain_bin}/arm-none-eabi-size")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
