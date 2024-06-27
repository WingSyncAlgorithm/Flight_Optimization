# declare the lowest version of cmake
cmake_minimum_required(VERSION 2.8)
# declare a cmake project
project(pcd_processing)
# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# eigen 
include_directories( "/usr/include/eigen3/" )

find_package(PCL 1.0 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# pcl 
find_package( PCL REQUIRED COMPONENT common io )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( joinMap joinMap.cpp )
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )

add_executable (view pcdviewer.cpp)
target_link_libraries (view ${PCL_LIBRARIES})

add_executable (oct toctree.cpp)
target_link_libraries (oct ${PCL_LIBRARIES})