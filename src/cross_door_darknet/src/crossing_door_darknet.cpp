#include <ros/ros.h>

// ROS 标准消息
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

// MAVROS 服务
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

// Darknet 目标检测
#include <darknet_ros_msgs/BoundingBoxes.h>

// OpenCV & 图像
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// 数学工具
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <vector>

using namespace std;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 全局变量 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0)

// 输入
sensor_msgs::LaserScan Laser;
geometry_msgs::PoseStamped pos_drone;
float final_x = 2.0;
float final_y = 0.0;
float door_x = 1.0;
float door_y = 1.7;
int range_min = 0;
int range_max = 359;

// 算法参数
float R_outside = 2.0, R_inside = 1.0;
float p_R = 0.0, p_r = 0.0;
float p_xy = 0.5;
float vel_track_max = 0.5;
float vel_collision_max = 0.5;
float vel_sp_max = 1.0;

float distance_c = 100.0, angle_c = 0.0;
float distance_cx = 0.0, distance_cy = 0.0;
float vel_collision[2] = {0, 0};
float vel_track[2] = {0, 0};
float vel_sp_ENU[2] = {0, 0};

bool reach_door_flag[2] = {false, false};
float fly_height = 1.5; // 默认飞行高度

// 目标检测相关
int detect_num = 0;
darknet_ros_msgs::BoundingBox darknet_box;
darknet_ros_msgs::BoundingBoxes darknet_boxes;

// 状态标志
bool flag_land = false;
bool flag_collision_avoidance = false;

// 飞控状态
bool armed = false;
bool offboard_enabled = false;

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 声明函数 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance();
float satfunc(float data, float Max);
void printf_status();
void printf_param();
void collision_avoidance(float target_x, float target_y);
void finddoorcentor(int i);
void detect_nav();

// 坐标系旋转：机体系速度 -> ENU 系（其实这里我们直接在 ENU 下计算，所以可能不需要）
void rotation_yaw(float yaw_angle, float input[2], float output[2]) {
    output[0] = input[0] * cos(yaw_angle) - input[1] * sin(yaw_angle);
    output[1] = input[0] * sin(yaw_angle) + input[1] * cos(yaw_angle);
}

// 四元数转欧拉角（来自 math_utils.h 的功能）
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) {
    Eigen::Vector3d euler;
    euler[0] = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                     1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    euler[1] = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
    euler[2] = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    return euler;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 回调函数 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void lidar_cb(const sensor_msgs::LaserScan::ConstPtr& scan) {
    Laser = *scan;
    int count = Laser.ranges.size();

    // 剔除 inf
    for (int i = 0; i < count; ++i) {
        if (isinf(Laser.ranges[i])) {
            Laser.ranges[i] = (i == 0) ? Laser.ranges[count - 1] : Laser.ranges[i - 1];
        }
    }

    // 激光角度翻转（适配你的坐标系）
    sensor_msgs::LaserScan tmp = Laser;
    for (int i = 0; i < count; ++i) {
        int src_idx = (i + 180) % count;
        Laser.ranges[i] = tmp.ranges[src_idx];
    }

    cal_min_distance();
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    pos_drone = *msg;
}

void darknet_found_cb(const std_msgs::Int8::ConstPtr& msg) {
    detect_num = msg->data;
}

void darknet_box_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    darknet_boxes = *msg;
    if (!msg->bounding_boxes.empty() && msg->bounding_boxes[0].Class == "person") {
        darknet_box = msg->bounding_boxes[0];
        ROS_INFO("Detected person!");
    }
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 主函数 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char** argv) {
    ros::init(argc, argv, "crossing_door_darknet");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);

    ros::Rate rate(20.0); // 20 Hz

    // 订阅
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, pos_cb);
    ros::Subscriber darknet_found_sub = nh.subscribe<std_msgs::Int8>("/darknet_ros/found_object", 1, darknet_found_cb);
    ros::Subscriber darknet_box_sub = nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, darknet_box_cb);

    // 发布器和服务
    ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // 读取参数
    nh.param<float>("R_outside", R_outside, 2.0);
    nh.param<float>("R_inside", R_inside, 1.0);
    nh.param<float>("p_xy", p_xy, 0.5);
    nh.param<float>("vel_track_max", vel_track_max, 0.5);
    nh.param<float>("p_R", p_R, 0.5);
    nh.param<float>("p_r", p_r, 1.0);
    nh.param<float>("vel_collision_max", vel_collision_max, 0.5);
    nh.param<float>("vel_sp_max", vel_sp_max, 1.0);
    nh.param<int>("range_min", range_min, 1);
    nh.param<int>("range_max", range_max, 359);
    nh.param<float>("final_x", final_x, 2.0);
    nh.param<float>("final_y", final_y, 0.0);
    nh.param<float>("door_x", door_x, 1.0);
    nh.param<float>("door_y", door_y, 1.7);
    nh.getParam("/px4_pos_controller/Takeoff_height", fly_height);

    printf_param();

    int check_flag;
    cout << "Check parameters. Enter 1 to continue, else quit: ";
    cin >> check_flag;
    if (check_flag != 1) return -1;

    // 预热 setpoint（PX4 要求）
    ROS_INFO("Sending zero velocity setpoints for 2 seconds...");
    geometry_msgs::Twist zero_cmd;
    for (int i = 0; i < 40; ++i) {
        velocity_pub.publish(zero_cmd);
        ros::Duration(0.05).sleep();
    }

    // Arm
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Drone ARMED.");
        armed = true;
    } else {
        ROS_ERROR("Failed to arm.");
        return -1;
    }

    // Set OFFBOARD mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD enabled.");
        offboard_enabled = true;
    } else {
        ROS_ERROR("Failed to set OFFBOARD mode.");
        return -1;
    }

    // 主循环
    while (ros::ok()) {
        ros::spinOnce();

        // ========== 穿门逻辑 ==========
        if (!reach_door_flag[0]) {
            collision_avoidance(door_x + 0.1, door_y);
            float dist = sqrt(pow(pos_drone.pose.position.x - door_x - 0.1, 2) +
                              pow(pos_drone.pose.position.y - door_y, 2));
            if (dist < 0.3) {
                reach_door_flag[0] = true;
                ROS_INFO("Reached first door!");
            }
        } else if (reach_door_flag[0] && !flag_land) {
            collision_avoidance(final_x, final_y);
            float dist = sqrt(pow(pos_drone.pose.position.x - final_x, 2) +
                              pow(pos_drone.pose.position.y - final_y, 2));
            if (dist < 0.3) {
                flag_land = true;
                ROS_INFO("Target reached. Initiating landing.");
            }
        }

        // ========== 发布速度指令 ==========
        geometry_msgs::Twist cmd;
        if (flag_land) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = -0.3; // 缓慢下降
        } else {
            cmd.linear.x = vel_sp_ENU[0];
            cmd.linear.y = vel_sp_ENU[1];
            cmd.linear.z = 0.0; // 保持高度（由飞控内部定高）
        }
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        velocity_pub.publish(cmd);

        printf_status();
        rate.sleep();
    }

    return 0;
}

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> 函数实现 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void cal_min_distance() {
    distance_c = 100.0;
    angle_c = 0.0;
    for (int i = range_min; i <= range_max; ++i) {
        if (Laser.ranges[i] < distance_c) {
            distance_c = Laser.ranges[i];
            angle_c = i; // degrees
        }
    }
}

float satfunc(float data, float Max) {
    if (abs(data) > Max) {
        return (data > 0) ? Max : -Max;
    }
    return data;
}

void collision_avoidance(float target_x, float target_y) {
    // 判断是否避障
    flag_collision_avoidance = (distance_c < R_outside);

    // 追踪速度
    vel_track[0] = p_xy * (target_x - pos_drone.pose.position.x);
    vel_track[1] = p_xy * (target_y - pos_drone.pose.position.y);
    vel_track[0] = satfunc(vel_track[0], vel_track_max);
    vel_track[1] = satfunc(vel_track[1], vel_track_max);

    // 初始化避障速度
    vel_collision[0] = 0.0;
    vel_collision[1] = 0.0;

    if (flag_collision_avoidance) {
        double angle_rad = DEG2RAD(angle_c);
        distance_cx = distance_c * cos(angle_rad);
        distance_cy = distance_c * sin(angle_rad);

        float F_c = 0.0;
        if (distance_c > R_inside) {
            F_c = p_R * (R_outside - distance_c);
        } else {
            F_c = p_R * (R_outside - R_inside) + p_r * (R_inside - distance_c);
        }

        // 推力方向远离障碍物
        vel_collision[0] = -F_c * (distance_cx / distance_c);
        vel_collision[1] = -F_c * (distance_cy / distance_c);

        vel_collision[0] = satfunc(vel_collision[0], vel_collision_max);
        vel_collision[1] = satfunc(vel_collision[1], vel_collision_max);
    }

    // 总速度（ENU）
    vel_sp_ENU[0] = vel_track[0] + vel_collision[0];
    vel_sp_ENU[1] = vel_track[1] + vel_collision[1];

    // 限幅
    vel_sp_ENU[0] = satfunc(vel_sp_ENU[0], vel_sp_max);
    vel_sp_ENU[1] = satfunc(vel_sp_ENU[1], vel_sp_max);
}

void printf_status() {
    static int counter = 0;
    if (++counter % 20 != 0) return; // 每秒打印一次

    ROS_INFO_STREAM("\n>>> Collision Avoidance Status <<<\n"
                    << "Min Distance: " << distance_c << " m @ " << angle_c << " deg\n"
                    << "Avoidance: " << (flag_collision_avoidance ? "ON" : "OFF") << "\n"
                    << "Vel Track: [" << vel_track[0] << ", " << vel_track[1] << "] m/s\n"
                    << "Vel Collision: [" << vel_collision[0] << ", " << vel_collision[1] << "] m/s\n"
                    << "Vel Command: [" << vel_sp_ENU[0] << ", " << vel_sp_ENU[1] << "] m/s\n"
                    << "Position: [" << pos_drone.pose.position.x << ", " << pos_drone.pose.position.y << "]\n"
                    << "Reach Door0: " << reach_door_flag[0] << ", Land: " << flag_land);
}

void printf_param() {
    ROS_INFO_STREAM(">>> Parameters <<<\n"
                    << "R_outside: " << R_outside << "\n"
                    << "R_inside: " << R_inside << "\n"
                    << "p_xy: " << p_xy << "\n"
                    << "p_R: " << p_R << "\n"
                    << "p_r: " << p_r << "\n"
                    << "vel limits: track=" << vel_track_max
                    << ", collision=" << vel_collision_max
                    << ", total=" << vel_sp_max << "\n"
                    << "Door: (" << door_x << ", " << door_y << ")\n"
                    << "Final: (" << final_x << ", " << final_y << ")\n"
                    << "Fly height: " << fly_height);
}
