cmake_minimum_required(VERSION 2.8.3)
project(lowLevelDetector)

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
)



catkin_package(
)

set(SOURCES
              src/cvFunctions.cpp
              src/cvFunctions.hpp
              src/main.cpp)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


include_directories(
                    ${catkin_INCLUDE_DIRS}
)


add_executable(low_level_detector_node src/main.cpp src/cvFunctions.cpp )

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries( low_level_detector_node 
                          ${catkin_LIBRARIES}
                          ${OpenCV_LIBRARIES})
