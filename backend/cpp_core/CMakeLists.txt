cmake_minimum_required(VERSION 3.16)
project(uav_digital_twin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenMP REQUIRED)

# Build shared library for Python ctypes FFI
add_library(digital_twin SHARED
    src/battery_ecm.cpp
    src/wind_kinematics.cpp
    src/ukf_estimator.cpp
    src/mc_rollout_engine.cpp
    src/c_api_bridge.cpp
)

target_include_directories(digital_twin PUBLIC include)
target_compile_options(digital_twin PRIVATE -O3 -Wall -Wextra)
target_link_libraries(digital_twin PUBLIC OpenMP::OpenMP_CXX)

# Standalone C++ verification testbench
add_executable(test_cpp_kernel tests/test_cpp_kernel.cpp)
target_link_libraries(test_cpp_kernel PRIVATE digital_twin)
