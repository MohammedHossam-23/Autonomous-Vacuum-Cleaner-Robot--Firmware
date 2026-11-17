# STM32F411CEUx firmware image, built with the arm-none-eabi toolchain file in
# cmake/arm-none-eabi.cmake. The source tree (including the CubeMX-generated
# Makefile project) is left untouched; this file only describes the same set of
# sources the CubeIDE build compiles.

set(FW ${PROJECT_SOURCE_DIR}/avcrLowLevelCotrol)

set(FW_APP_SOURCES ${FW}/Core/Src)
set(FW_DRIVER_INC
    ${FW}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${FW}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
)
set(FW_CMSIS_INC
    ${FW}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${FW}/Drivers/CMSIS/Include
)
set(FW_FREERTOS_DIR ${FW}/Middlewares/Third_Party/FreeRTOS/Source)
set(FW_FREERTOS_INC
    ${FW_FREERTOS_DIR}/include
    ${FW_FREERTOS_DIR}/CMSIS_RTOS_V2
    ${FW_FREERTOS_DIR}/portable/GCC/ARM_CM4F
)

file(GLOB FW_CORE_SOURCES ${FW_APP_SOURCES}/*.c)
file(GLOB FW_HAL_SOURCES ${FW}/Drivers/STM32F4xx_HAL_Driver/Src/*.c)

add_executable(avcr_firmware
    ${FW_CORE_SOURCES}
    ${FW}/Core/Startup/startup_stm32f411ceux.s
    ${FW_HAL_SOURCES}
    ${FW_FREERTOS_DIR}/tasks.c
    ${FW_FREERTOS_DIR}/queue.c
    ${FW_FREERTOS_DIR}/list.c
    ${FW_FREERTOS_DIR}/timers.c
    ${FW_FREERTOS_DIR}/event_groups.c
    ${FW_FREERTOS_DIR}/stream_buffer.c
    ${FW_FREERTOS_DIR}/croutine.c
    ${FW_FREERTOS_DIR}/CMSIS_RTOS_V2/cmsis_os2.c
    ${FW_FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c
    ${FW_FREERTOS_DIR}/portable/MemMang/heap_4.c
)

target_include_directories(avcr_firmware PRIVATE
    ${FW}/Core/Inc
    ${FW_DRIVER_INC}
    ${FW_CMSIS_INC}
    ${FW_FREERTOS_INC}
)

target_compile_definitions(avcr_firmware PRIVATE
    USE_HAL_DRIVER
    STM32F411xE
)

target_compile_options(avcr_firmware PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -std=gnu11
    -ffunction-sections
    -fdata-sections
    -Wall
    -Wextra
    -Wno-unused-parameter
)

target_link_options(avcr_firmware PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -T${FW}/STM32F411CEUX_FLASH.ld
    --specs=nano.specs
    --specs=nosys.specs
    -Wl,--gc-sections
    # CubeMX's linker script places .init_array/.fini_array in FLASH with the
    # writable flag, so ld emits an RWX-segment warning. Harmless here.
    -Wl,--no-warn-rwx-segments
    -Wl,-Map=${CMAKE_BINARY_DIR}/avcr_firmware.map,--cref
    -Wl,--print-memory-usage
)

target_link_libraries(avcr_firmware PRIVATE m)

set_target_properties(avcr_firmware PROPERTIES
    OUTPUT_NAME avcr_firmware
    SUFFIX ".elf"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
    LINK_DEPENDS ${FW}/STM32F411CEUX_FLASH.ld
)

add_custom_command(TARGET avcr_firmware POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:avcr_firmware>
            ${CMAKE_BINARY_DIR}/avcr_firmware.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:avcr_firmware>
            ${CMAKE_BINARY_DIR}/avcr_firmware.hex
    COMMENT "Generating avcr_firmware.bin and avcr_firmware.hex"
)
