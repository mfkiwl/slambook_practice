cmake_minimum_required(VERSION 3.4)
project(lie_algebra_practice)

add_compile_options(-g -Werror -Wall)

include_directories("/home/jeff/opensource/eigen/")

find_package(Sophus REQUIRED)

add_executable(useSophus useSophus.cpp)

target_link_libraries(useSophus Sophus::Sophus)

