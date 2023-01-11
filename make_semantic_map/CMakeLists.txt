cmake_minimum_required(VERSION 3.10)
project(make_semantic_map)

set(CMAKE_CXX_STANDARD 17)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

FIND_PACKAGE( Boost 1.65 COMPONENTS program_options REQUIRED )

add_executable(save_semantic_map save_semantic_map.cpp)
target_link_libraries (save_semantic_map ${PCL_LIBRARIES} ${Boost_LIBRARIES})