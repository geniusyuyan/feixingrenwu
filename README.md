更改color_landing.py里的摄像头话题
放在工作空间下编译

输入
source devel/setup.bash
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot:~/PX4-Autopilot/Tools/sitl_gazebo

加载地图
roslaunch sim_task task.launch

起飞
roslaunch template template.launch
