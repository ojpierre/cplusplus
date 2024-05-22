cmake_minimum_required(VERSION 3.10)

project(cplusplus)

# Use vcpkg toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "Vcpkg toolchain file")

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Add executables
add_executable(main src/main.cpp)
add_executable(test_graph tests/test_graph.cpp)

# Find and link Google Test
find_package(GTest REQUIRED)
target_link_libraries(test_graph GTest::gtest GTest::gtest_main)
enable_testing()
add_test(NAME test_graph COMMAND test_graph)
