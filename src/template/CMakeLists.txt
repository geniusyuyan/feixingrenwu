cmake_minimum_required(VERSION 3.0.2)
project(template)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  message_generation
  color_landing  # 添加这一行
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  mavros_msgs
  color_landing  # 添加这一行
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(template_node src/template.cpp)

target_link_libraries(template_node
  ${catkin_LIBRARIES}
)

add_dependencies(template_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  color_landing_generate_messages_cpp  # 添加这一行，确保先生成消息
)

install(TARGETS template_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
