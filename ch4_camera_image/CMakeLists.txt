cmake_minimum_required(VERSION 3.4)
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-g -Werror -Wall)

include_directories("/home/jeff/opensource/eigen/")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRES})

add_subdirectory(imagebasics)
add_subdirectory(3dvision)


