cmake_minimum_required(VERSION 3.4)
project(lie_algebra_practice)

add_compile_options(-g -Werror -Wall)

include_directories("/home/jeff/opensource/eigen/")

find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRES})

add_executable(useSophus useSophus.cpp)
add_executable(Trajectory_dif Trajectory_dif.cpp)
add_executable(practice_exp_log_map practice_exp_log_map.cpp)

target_link_libraries(useSophus Sophus::Sophus)
target_link_libraries(Trajectory_dif Sophus::Sophus ${Pangolin_LIBRARIES})
target_link_libraries(practice_exp_log_map Sophus::Sophus ${Pangolin_LIBRARIES})

