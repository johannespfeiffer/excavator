add_executable(excavator_math_tests
    test_kinematics.c
)

target_link_libraries(excavator_math_tests PRIVATE excavator_core)
target_compile_options(excavator_math_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_math_tests COMMAND $<TARGET_FILE:excavator_math_tests>)

add_executable(excavator_orientation_tests
    test_orientation.c
)

target_link_libraries(excavator_orientation_tests PRIVATE excavator_core)
target_compile_options(excavator_orientation_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_orientation_tests COMMAND $<TARGET_FILE:excavator_orientation_tests>)

add_executable(excavator_calibration_tests
    test_calibration.c
)

target_link_libraries(excavator_calibration_tests PRIVATE excavator_core)
target_compile_options(excavator_calibration_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_calibration_tests COMMAND $<TARGET_FILE:excavator_calibration_tests>)

add_executable(excavator_telemetry_tests
    test_telemetry.c
)

target_link_libraries(excavator_telemetry_tests PRIVATE excavator_core)
target_compile_options(excavator_telemetry_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_telemetry_tests COMMAND $<TARGET_FILE:excavator_telemetry_tests>)

add_executable(excavator_app_tests
    test_app.c
)

target_link_libraries(excavator_app_tests PRIVATE excavator_core)
target_compile_options(excavator_app_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_app_tests COMMAND $<TARGET_FILE:excavator_app_tests>)

add_executable(excavator_bmi160_tests
    test_bmi160.c
)

target_link_libraries(excavator_bmi160_tests PRIVATE excavator_core)
target_compile_options(excavator_bmi160_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_bmi160_tests COMMAND $<TARGET_FILE:excavator_bmi160_tests>)

add_executable(excavator_gps_tests
    test_gps_parser.c
)

target_link_libraries(excavator_gps_tests PRIVATE excavator_core)
target_compile_options(excavator_gps_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_gps_tests COMMAND $<TARGET_FILE:excavator_gps_tests>)

add_executable(excavator_state_tests
    test_excavator_state.c
)

target_link_libraries(excavator_state_tests PRIVATE excavator_core)
target_compile_options(excavator_state_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_state_tests COMMAND $<TARGET_FILE:excavator_state_tests>)

add_executable(excavator_platform_tests
    test_platform.c
)

target_link_libraries(excavator_platform_tests PRIVATE excavator_core)
target_compile_options(excavator_platform_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_platform_tests COMMAND $<TARGET_FILE:excavator_platform_tests>)

add_executable(excavator_platform_io_tests
    test_platform_io.c
)

target_link_libraries(excavator_platform_io_tests PRIVATE excavator_core)
target_compile_options(excavator_platform_io_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_platform_io_tests COMMAND $<TARGET_FILE:excavator_platform_io_tests>)

add_executable(excavator_config_tests
    test_config.c
)

target_link_libraries(excavator_config_tests PRIVATE excavator_core)
target_compile_options(excavator_config_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_config_tests COMMAND $<TARGET_FILE:excavator_config_tests>)
