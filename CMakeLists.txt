cmake_minimum_required(VERSION 3.10)
project(transform
    VERSION 0.1.0
    LANGUAGES CXX)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(transform transform.cpp)
target_compile_options(transform PRIVATE -Wall -Wextra)
target_link_libraries(transform PRIVATE Threads::Threads)
set_property(TARGET transform PROPERTY CXX_STANDARD 17)
