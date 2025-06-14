# Add test files
set(TEST_FILES test_timing.cpp)

# Create test executable
add_executable(test_timing ${TEST_FILES})

# Link Google Test and your project
target_link_libraries(test_timing gtest gtest_main)
target_link_libraries(test_timing MPC)
target_link_libraries(test_timing Autodiff)

include_directories(/home/noelcs/repos/CppAD_install/include)
include_directories(/home/noelcs/repos/CppADCodeGen_install/include)

target_link_libraries(test_timing /home/noelcs/repos/CppAD_install/lib/libcppad_lib.so)

target_compile_definitions(test_timing PUBLIC PINOCCHIO_CXX_COMPILER=\"${CMAKE_CXX_COMPILER}\")

# Add tests to CTest
add_test(NAME Tests COMMAND test_timing)
