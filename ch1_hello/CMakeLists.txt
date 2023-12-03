cmake_minimum_required(VERSION 2.8)
project(hello)

add_compile_options(-g -Werror -Wall)


add_library(libhello SHARED libhello.cpp)


add_executable(hello hello.cpp)


target_link_libraries(hello libhello)