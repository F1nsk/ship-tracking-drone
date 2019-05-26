cmake_minimum_required(VERSION 2.8.3)
project(stateMachine)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge  
  sensor_msgs

)


catkin_package(
)

set(SOURCES
              src/stateMachine.cpp
              src/stateMachine.hpp
              src/main.cpp)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


include_directories(
                    ${catkin_INCLUDE_DIRS}
)


add_executable(stateMachine_node src/main.cpp src/stateMachine.cpp )


find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries( stateMachine_node  
                          ${catkin_LIBRARIES}
                          ${OpenCV_LIBRARIES})