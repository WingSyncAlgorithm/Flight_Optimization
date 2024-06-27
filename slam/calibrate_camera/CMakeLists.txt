cmake_minimum_required(VERSION 2.8)

project(calibrate_camera)

# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( cali cali.cpp )
target_link_libraries( cali ${OpenCV_LIBS} )
add_executable( rect rectify.cpp)
target_link_libraries( rect ${OpenCV_LIBS} )
add_executable( match match.cpp)
target_link_libraries( match ${OpenCV_LIBS} )
