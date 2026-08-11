# This is only used for profiling with Linux perf tool

cmake_minimum_required(VERSION 3.16)
project(fast_pointcloud CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

add_library(fast_pointcloud MODULE
    src/fast_pointcloud/module.cpp
    src/fast_pointcloud/pcdecode.cpp
)

target_include_directories(fast_pointcloud PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src/utils
)

target_link_libraries(fast_pointcloud PRIVATE Python3::Module)


target_compile_options(fast_pointcloud PRIVATE
    -std=c++20
    -O3
    -fno-omit-frame-pointer
    -flto
)

target_link_options(fast_pointcloud PRIVATE
    -flto
)


