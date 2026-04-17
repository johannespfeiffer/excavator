add_executable(excavator_math_tests
    test_kinematics.c
)

target_link_libraries(excavator_math_tests PRIVATE excavator_core)
target_compile_options(excavator_math_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_math_tests COMMAND $<TARGET_FILE:excavator_math_tests>)

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

add_executable(excavator_platform_tests
    test_platform.c
)

target_link_libraries(excavator_platform_tests PRIVATE excavator_core)
target_compile_options(excavator_platform_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_platform_tests COMMAND $<TARGET_FILE:excavator_platform_tests>)

add_executable(excavator_config_tests
    test_config.c
)

target_link_libraries(excavator_config_tests PRIVATE excavator_core)
target_compile_options(excavator_config_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_config_tests COMMAND $<TARGET_FILE:excavator_config_tests>)
