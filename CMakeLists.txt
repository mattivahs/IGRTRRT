cmake_minimum_required(VERSION 3.0.2)
project(ipprrt)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)



include_directories(
  include ${catkin_INCLUDE_DIRS}
)
catkin_package()

add_executable(IPPNode nodes/IPPNode.cpp nodes/GMRF.h nodes/GMRF.cpp nodes/helper_functions.h nodes/helper_functions.cpp
nodes/obstacle.h nodes/obstacle.cpp nodes/rrt_star.h nodes/rrt_star.cpp 
nodes/node.h nodes/node.cpp)
target_link_libraries(IPPNode ${catkin_LIBRARIES})



