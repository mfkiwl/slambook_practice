cmake_minimum_required(VERSION 2.8)
project(3d_rigid_motion)

add_compile_options(-g -Werror -Wall)

include_directories("/home/jeff/opensource/eigen/")

find_package( Pangolin )
include_directories( ${Pangolin_INCLUDE_DIRS} )

add_executable(use_eigen use_eigen.cpp)
add_executable(geometry use_geometry.cpp)
add_executable(transform coordinate_transfrom.cpp)
add_executable(visualize_trajectory visualize_trajectory.cpp)
add_executable(visualize_geometry visualize_geometry.cpp)

target_link_libraries(visualize_trajectory ${Pangolin_LIBRARIES} )
target_link_libraries(visualize_geometry ${Pangolin_LIBRARIES} )