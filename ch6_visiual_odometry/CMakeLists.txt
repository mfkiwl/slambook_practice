cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-g -Wall -Werror)

include_directories("/home/jeff/opensource/eigen/")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(orb_cv orb_cv.cpp)
add_executable(pose_estimated_2d2d pose_estimated_2d2d.cpp)

target_link_libraries(orb_cv ${OpenCV_LIBS})
target_link_libraries(pose_estimated_2d2d ${OpenCV_LIBS})