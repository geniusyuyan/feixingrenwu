cmake_minimum_required(VERSION 3.0.2)
project(color_landing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)

# 删除或注释掉下面这行：
# catkin_python_setup()

# 然后继续其他配置
add_service_files(
  FILES
  RememberColor.srv
  FindTarget.srv
  GetColor.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  rospy 
  std_msgs 
  sensor_msgs 
  geometry_msgs 
  message_runtime
)

# 安装Python脚本
install(PROGRAMS
  scripts/color_memory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/color_landing.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY
  srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
