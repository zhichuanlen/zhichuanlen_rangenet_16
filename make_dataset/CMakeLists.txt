cmake_minimum_required(VERSION 3.10)
project(make_dataset)

set(CMAKE_CXX_STANDARD 17)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE( Boost 1.65 COMPONENTS program_options REQUIRED )

add_executable(${PROJECT_NAME}_make_dataset make_dataset.cpp)
target_link_libraries (${PROJECT_NAME}_make_dataset ${PCL_LIBRARIES} ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_bin_pcd_cov bin_pcd_cov.cpp)
target_link_libraries (${PROJECT_NAME}_bin_pcd_cov ${PCL_LIBRARIES} ${Boost_LIBRARIES})