cmake_minimum_required(VERSION 2.8.3)
project(tracker)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  darknet_ros_msgs
)


catkin_package(
)

set(SOURCES
              src/tracker.cpp
              src/tracker.hpp
              src/main.cpp)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


include_directories(
                    ${catkin_INCLUDE_DIRS}
)


add_executable(tracker_node src/main.cpp src/tracker.cpp )
target_link_libraries( tracker_node 
                          ${catkin_LIBRARIES}
                          )
