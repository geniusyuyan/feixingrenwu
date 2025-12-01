cmake_minimum_required(VERSION 3.0.2)
project(cross_door_darknet)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  mavros_msgs
  eigen_conversions
  cv_bridge
  image_transport
  message_generation
  darknet_ros_msgs
)
catkin_package(
  INCLUDE_DIRS include
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_executable(crossing_door_darknet_node src/crossing_door_darknet.cpp)

add_dependencies(crossing_door_darknet_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  darknet_ros_msgs_generate_messages_cpp
)

target_link_libraries(crossing_door_darknet_node ${catkin_LIBRARIES} )
