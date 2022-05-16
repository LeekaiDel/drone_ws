cmake_minimum_required(VERSION 3.0.2)
project(pcl_programs)

find_package(catkin REQUIRED COMPONENTS
  # PCL
  pcl_ros
  pcl_conversions
  roscpp
  laser_geometry
)
# find_package(PCL 1.10 REQUIRED)
find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES pcl_frame_detector
 CATKIN_DEPENDS pcl roscpp
 DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories( ${PCL_LIBRARY_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

# add_executable(pcl_scan_converter src/pcl_laser_scan.cpp)
# target_link_libraries(pcl_scan_converter ${catkin_LIBRARIES})

add_executable(pcl src/pcl_camera_eater.cpp)
target_link_libraries(pcl ${catkin_LIBRARIES})