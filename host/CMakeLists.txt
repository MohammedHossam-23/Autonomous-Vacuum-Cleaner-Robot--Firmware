set(AVCR_ROOT ${PROJECT_SOURCE_DIR})
set(FW ${AVCR_ROOT}/avcrLowLevelCotrol)

# Firmware modules that are portable enough to run on the host: COBS, the PID
# controller, odometry (through the HAL shim) and nanopb with the generated
# robot messages.
add_library(avcr_core STATIC
    ${FW}/Core/Src/cobs.c
    ${FW}/Core/Src/PID.c
    ${FW}/Core/Src/timerEncoderR.c
    ${FW}/Core/Src/pb_common.c
    ${FW}/Core/Src/pb_decode.c
    ${FW}/Core/Src/pb_encode.c
    ${FW}/Core/Src/rx.pb.c
    ${FW}/Core/Src/tx.pb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/crc32_mpeg2.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frame.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/host_hal.c
)

target_include_directories(avcr_core PUBLIC
    ${FW}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/compat
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(avcr_core PRIVATE
    -Wall -Wextra
    -Wno-unused-parameter
    -Wno-sign-compare
)

target_link_libraries(avcr_core PUBLIC m)

add_executable(avcr_cli cli/main.c)
set_target_properties(avcr_cli PROPERTIES
    OUTPUT_NAME avcr-cli
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
target_link_libraries(avcr_cli PRIVATE avcr_core)
target_compile_options(avcr_cli PRIVATE -Wall -Wextra)
