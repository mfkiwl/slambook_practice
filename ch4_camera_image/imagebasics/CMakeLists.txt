project(imagebasics)

add_executable(imagebasics imagebasics.cpp)
add_executable(undistored undistored.cpp)
target_link_libraries(imagebasics ${OpenCV_LIBS})
target_link_libraries(undistored ${OpenCV_LIBS})