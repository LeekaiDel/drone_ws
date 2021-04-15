cmake_minimum_required(VERSION 3.0.2)
project(pcl_frame_detector)

find_package(catkin REQUIRED COMPONENTS
  PCL
  pcl_ros
  pcl_conversions
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_frame_detector
#  CATKIN_DEPENDS pcl roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions( ${PCL_DEFINITIONS} )
link_directories( ${PCL_LIBRARY_DIRS} )

add_executable(pcl_visualizer src/pcl_visualization.cpp)
target_link_libraries(pcl_visualizer ${catkin_LIBRARIES})

add_executable(pcl src/pcl.cpp)
target_link_libraries(pcl ${catkin_LIBRARIES})