cmake_minimum_required(VERSION 3.10)
project(make_semantic_map)

set(CMAKE_CXX_STANDARD 17)
find_package(PCL 1.2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rosbag
  geometry_msgs
  nav_msgs
  pcl_ros
  sensor_msgs
)
include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE( Boost 1.65 COMPONENTS program_options REQUIRED )

add_executable(save_semantic_map save_semantic_map.cpp)
target_link_libraries (save_semantic_map ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(get_ppm_from_pcd get_ppm_from_pcd.cpp)
target_link_libraries (get_ppm_from_pcd ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})