cmake_minimum_required(VERSION 3.4)
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-g -Werror -Wall)

include_directories("/home/jeff/opensource/eigen/")
LIST( APPEND CMAKE_MODULE_PATH /usr/cmake_module ) #for g2o cmake modules

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED)
find_package(G2O REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRES})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})

add_executable(gauss_newton gauss_newton.cpp)
add_executable(ceres_curve_fitting ceres_curve_fitting.cpp)
add_executable(g2o_curve_fitting g2o_curve_fitting.cpp)


target_link_libraries(gauss_newton )
target_link_libraries(ceres_curve_fitting ${CERES_LIBRARIES})
target_link_libraries(g2o_curve_fitting ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})