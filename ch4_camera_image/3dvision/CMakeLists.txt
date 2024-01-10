project(3dvision)

add_executable(stereo stereo.cpp)
add_executable(rgbd rgbd.cpp)

target_link_libraries(stereo ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
target_link_libraries(rgbd ${OpenCV_LIBS} ${Pangolin_LIBRARIES} Sophus::Sophus)
