#include <ros/ros.h>
#include <template.h>  // 确保包含 state_cb, local_pos_cb, mission_pos_cruise, precision_land, setpoint_raw, current_state, local_pos, ALTITUDE 等定义
#include <color_landing/RememberColor.h>
#include <color_landing/FindTarget.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <sensor_msgs/LaserScan.h>  // 新增
#include <cmath>
#include <map>
#include <vector>
#include <cstdlib>
#include <iostream>

// 全局变量定义
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;

// 颜色识别服务客户端
ros::ServiceClient remember_color_client;
ros::ServiceClient find_target_client;

// 目标点坐标：颜色ID -> {x, y}
std::map<int, std::vector<float>> target_points = {
    {1, {35, -1}},   // 红色降落点
    {2, {35, 1}},    // 黄色降落点  
    {3, {35, 3}}     // 蓝色降落点
};

int target_color_id = 0;  // 目标颜色ID

// ===== 新增：激光避障相关 =====
std::vector<float> laser_ranges(360, 10.0); // /laser/scan 是 360 度
float drone_x = 0.0, drone_y = 0.0;

void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if (msg->ranges.size() != 360) return;
    for (size_t i = 0; i < 360; ++i) {
        float r = msg->ranges[i];
        laser_ranges[i] = (!std::isfinite(r) || r > 10.0f) ? 10.0f : r;
    }
}

float satfunc(float x, float max_val) {
    return std::max(-max_val, std::min(max_val, x));
}

void check_obstacle(int angle_deg, float &vx, float &vy, float safe_dist) {
    float r = laser_ranges[angle_deg];
    if (r < safe_dist) {
        float theta = (angle_deg) * M_PI / 180.0f;
        float obs_x = std::cos(theta);
        float obs_y = std::sin(theta);
        float repel = (safe_dist - r) * 1.0f;
        vx -= obs_x * repel;
        vy -= obs_y * repel;
    }
}

void get_avoidance_velocity(float target_x, float target_y, float &vx, float &vy) {
    drone_x = local_pos.pose.pose.position.x;
    drone_y = local_pos.pose.pose.position.y;

    float dx = target_x - drone_x;
    float dy = target_y - drone_y;
    float dist = std::sqrt(dx*dx + dy*dy);

    float base_speed = (dist > 1.0) ? 0.6f : 0.3f;
    vx = (dist > 0.1f) ? (dx / dist) * base_speed : 0.0f;
    vy = (dist > 0.1f) ? (dy / dist) * base_speed : 0.0f;

    const float SAFE_DIST = 1.2f;
    for (int ang = 315; ang < 360; ++ang) check_obstacle(ang, vx, vy, SAFE_DIST);
    for (int ang = 0; ang <= 45; ++ang)  check_obstacle(ang, vx, vy, SAFE_DIST);
}

bool mission_pos_cruise_avoid(float x, float y, float z, float yaw, float err_max) {
    float avoid_vx = 0.0, avoid_vy = 0.0;
    get_avoidance_velocity(x, y, avoid_vx, avoid_vy);

    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.type_mask = 0b0000000111111000; // POSITION + VELOCITY
    setpoint_raw.position.x = x;
    setpoint_raw.position.y = y;
    setpoint_raw.position.z = z;
    setpoint_raw.yaw = yaw;
    setpoint_raw.velocity.x = avoid_vx;
    setpoint_raw.velocity.y = avoid_vy;
    setpoint_raw.velocity.z = 0.0;

    float dx = local_pos.pose.pose.position.x - x;
    float dy = local_pos.pose.pose.position.y - y;
    float dz = local_pos.pose.pose.position.z - z;
    return (std::sqrt(dx*dx + dy*dy + dz*dz) < err_max);
}
// ===== 激光避障结束 =====

bool call_remember_color()
{
    color_landing::RememberColor srv;
    if (remember_color_client.call(srv)) {
        ROS_INFO("颜色记忆: %s", srv.response.message.c_str());
        return srv.response.success;
    } else {
        ROS_ERROR("颜色记忆服务调用失败");
        return false;
    }
}

int call_find_target()
{
    color_landing::FindTarget srv;
    if (find_target_client.call(srv)) {
        ROS_INFO("目标寻找: %s", srv.response.message.c_str());
        return srv.response.color_id;
    } else {
        ROS_ERROR("目标寻找服务调用失败");
        return 0;
    }
}

void print_param()
{
    std::cout << "=== 控制参数 ===" << std::endl;
    std::cout << "err_max: " << err_max << std::endl;
    std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
    std::cout << "if_debug: " << if_debug << std::endl;
    if(if_debug == 1) 
        std::cout << "自动offboard" << std::endl;
    else 
        std::cout << "遥控器offboard" << std::endl;
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "template");
    ros::NodeHandle nh;

    // 订阅 MAVROS 话题
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    // ===== 新增：订阅激光 =====
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/laser/scan", 10, laser_cb);

    // 发布控制指令
    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

    // 服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    remember_color_client = nh.serviceClient<color_landing::RememberColor>("/color_landing/remember_color");
    find_target_client = nh.serviceClient<color_landing::FindTarget>("/color_landing/find_target");

    ros::Rate rate(20);

    // 读取参数
    nh.param<float>("err_max", err_max, 0.2);
    nh.param<float>("if_debug", if_debug, 0.0);
    print_param();

    int choice = 0;
    std::cout << "1 to go on, else to quit: ";
    std::cin >> choice;
    if (choice != 1) return 0;

    ros::spinOnce();
    rate.sleep();

    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接");

    setpoint_raw.type_mask = 960; // 64+128+256+512
    setpoint_raw.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "ok" << std::endl;

    ros::Time last_request = ros::Time::now();

    // === 起飞准备阶段 ===
    ROS_INFO("=== 起飞准备阶段 ===");
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }

        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
        mavros_setpoint_pos_pub.publish(setpoint_raw);

        if (std::fabs(local_pos.pose.pose.position.z - ALTITUDE) < err_max)
        {
            ROS_INFO("已到达目标高度 %.2f m，进入任务阶段", ALTITUDE);
            mission_num = 1;
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    // === 任务执行阶段 ===
    ROS_INFO("=== 任务执行阶段 ===");
    last_request = ros::Time::now();

    while (ros::ok())
    {
        ROS_WARN_THROTTLE(1.0, "mission_num = %d", mission_num);

        static int search_step = 0;
        float search_points[3][2] = {{35, -2}, {35, 0}, {35, 2}};

        switch (mission_num)
        {
            case 1:
                ROS_INFO_ONCE("任务1: 起飞悬停");
                if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
                {
                    mission_num = 2;
                    last_request = ros::Time::now();
                }
                break;

            case 2:
                ROS_INFO_ONCE("任务2: 记忆起始点颜色...");
                if (call_remember_color()) {
                    mission_num = 3;
                    last_request = ros::Time::now();
                } else if (ros::Time::now() - last_request >= ros::Duration(5.0)) {
                    mission_num = 3;
                    last_request = ros::Time::now();
                }
                break;

            case 3:
                ROS_INFO_ONCE("任务3: 飞往穿门前 (20, 0.2)");
                if (mission_pos_cruise_avoid(20.0, 0.2, ALTITUDE, 0.0, err_max))
                {
                    mission_num = 4;
                    last_request = ros::Time::now();
                }
                break;

            case 4:
                ROS_INFO_ONCE("任务4: 穿门 (24, 0.2)");
                if (mission_pos_cruise_avoid(24.0, 0.2, ALTITUDE, 0.0, err_max))
                {
                    ROS_INFO("成功穿门！");
                    mission_num = 5;
                    last_request = ros::Time::now();
                }
                break;

            case 5:
                ROS_INFO_ONCE("任务5: 飞往目标区域 (35, 0)");
                if (mission_pos_cruise_avoid(35.0, 0.0, ALTITUDE, 0.0, err_max))
                {
                    mission_num = 6;
                    last_request = ros::Time::now();
                }
                break;

            case 6:
                ROS_INFO_ONCE("任务6: 寻找匹配降落点...");
                {
                    int target_id = call_find_target();
                    if (target_id > 0) {
                        target_color_id = target_id;
                        mission_num = 7;
                        last_request = ros::Time::now();
                    } else if (ros::Time::now() - last_request >= ros::Duration(8.0)) {
                        mission_num = 8;
                        search_step = 0;
                        last_request = ros::Time::now();
                    }
                }
                // 使用普通巡航（无避障）
                mission_pos_cruise(35.0, 0.0, ALTITUDE, 0.0, err_max);
                break;

            case 7:
                if (target_color_id > 0 && target_points.count(target_color_id)) {
                    auto point = target_points[target_color_id];
                    ROS_INFO_ONCE("任务7: 飞往颜色%d降落点 (%.1f, %.1f)", target_color_id, point[0], point[1]);
                    if (mission_pos_cruise(point[0], point[1], ALTITUDE, 0.0, err_max)) {
                        mission_num = 9;
                        last_request = ros::Time::now();
                    }
                } else {
                    mission_num = 8;
                    search_step = 0;
                }
                break;

            case 8:
                ROS_INFO_ONCE("任务8: 搜索模式 - 步骤 %d", search_step + 1);
                if (search_step < 3) {
                    if (mission_pos_cruise(search_points[search_step][0], search_points[search_step][1], ALTITUDE, 0.0, err_max)) {
                        int target_id = call_find_target();
                        if (target_id > 0) {
                            target_color_id = target_id;
                            mission_num = 7;
                        } else {
                            search_step++;
                            last_request = ros::Time::now();
                        }
                    }
                } else {
                    ROS_ERROR("搜索失败");
                    mission_num = -1;
                }
                break;

            case 9:
                ROS_INFO_ONCE("任务9: 开始降落");
                if (precision_land()) {
                    mission_num = -1;
                }
                break;

            default:
                if (mission_num == -1) {
                    ROS_INFO("=== 任务完成! ===");
                    return 0;
                }
                break;
        }

        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
