cmake_minimum_required(VERSION 3.10)
project(transform
    VERSION 0.1.0
    LANGUAGES CXX)

add_executable(transform transform.cpp)
target_compile_options(transform PRIVATE -Wall -Wextra)
set_property(TARGET transform PROPERTY CXX_STANDARD 17)
