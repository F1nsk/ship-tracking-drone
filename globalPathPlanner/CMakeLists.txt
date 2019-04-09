cmake_minimum_required(VERSION 2.8.3)
project(globalPathPlanner)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package(
)

set(SOURCES
                ship-tracking-drone/src/globalPathPlanner.cpp
                ship-tracking-drone/src/globalPathPlanner.hpp
                ship-tracking-drone/src/main.cpp)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


include_directories(
                    ${catkin_INCLUDE_DIRS}
)


add_executable(globalPathPlanner_node src/main.cpp src/globalPathPlanner.cpp)
target_link_libraries( globalPathPlanner_node 
                          ${catkin_LIBRARIES}
                          )
