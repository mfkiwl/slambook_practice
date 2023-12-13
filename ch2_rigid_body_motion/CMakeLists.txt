cmake_minimum_required(VERSION 2.8)
project(3d_rigid_motion)

add_compile_options(-g -Werror -Wall)

include_directories("/home/jeff/opensource/eigen/")

add_executable(use_eigen use_eigen.cpp)
add_executable(geometry use_geometry.cpp)
