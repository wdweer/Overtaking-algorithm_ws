cmake_minimum_required(VERSION 3.0.2)
project(Overtaking_algorithm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  autoware_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  hmcl_msgs
)

## Declare a catkin package
catkin_package(
#   INCLUDE_DIRS include
  LIBRARIES Overtaking_algorithm
  CATKIN_DEPENDS roscpp geometry_msgs autoware_msgs visualization_msgs tf
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/Overtaking_Algorithm.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)