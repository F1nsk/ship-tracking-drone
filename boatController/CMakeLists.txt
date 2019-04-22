cmake_minimum_required(VERSION 2.8.3)
project(boatController)

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
                boatController/src/boatController.cpp
                boatController/src/boatController.hpp
                boatController/src/main.cpp)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


include_directories(
                    ${catkin_INCLUDE_DIRS}
)


add_executable(boatController_node src/main.cpp src/boatController.cpp)
target_link_libraries(boatController_node 
                          ${catkin_LIBRARIES}
                          )
