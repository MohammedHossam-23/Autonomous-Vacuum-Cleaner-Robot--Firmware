set(AVCR_TESTS
    test_cobs
    test_frame
    test_pid
    test_odometry
    test_proto
)

foreach(test_name IN LISTS AVCR_TESTS)
    add_executable(${test_name} ${test_name}.c)
    target_link_libraries(${test_name} PRIVATE avcr_core)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(${test_name} PRIVATE -Wall -Wextra)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
