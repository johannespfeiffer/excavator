add_executable(excavator_math_tests
    test_kinematics.c
)

target_link_libraries(excavator_math_tests PRIVATE excavator_core)
target_compile_options(excavator_math_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME excavator_math_tests COMMAND $<TARGET_FILE:excavator_math_tests>)
