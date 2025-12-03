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

int landing_state = 0;
const float SAFE_LAND_X = 35.0f;
const float SAFE_LAND_Y = 0.0f;

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

// ===== 激光避障相关 =====
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
        
        float repel = (safe_dist - r) * 2.0f;
        vx -= obs_x * repel;
        vy -= obs_y * repel;
    }
}

// ✅ 已优化：降速 + 扩展避障角度 + 速度限幅
void get_avoidance_velocity(float target_x, float target_y, float &vx, float &vy) {
    drone_x = local_pos.pose.pose.position.x;
    drone_y = local_pos.pose.pose.position.y;

    float dx = target_x - drone_x;
    float dy = target_y - drone_y;
    float dist = std::sqrt(dx*dx + dy*dy);

    // 分段降速：大幅降低飞行速度，减少惯性冲撞
    float base_speed = 0.0f;
    if (dist > 2.0f) {
        base_speed = 0.35f;   // 远距离
    } else if (dist > 0.8f) {
        base_speed = 0.25f;   // 中距离
    } else {
        base_speed = 0.15f;   // 近距离精细调整
    }

    if (dist > 0.1f) {
        vx = (dx / dist) * base_speed;
        vy = (dy / dist) * base_speed;
    } else {
        vx = 0.0f;
        vy = 0.0f;
    }

    // 更保守的避障：安全距离增大，检测范围扩大到 ±90°
    const float SAFE_DIST = 1.5f;
    for (int ang = 270; ang < 360; ++ang) check_obstacle(ang, vx, vy, SAFE_DIST);
    for (int ang = 0; ang <= 90; ++ang)  check_obstacle(ang, vx, vy, SAFE_DIST);

    // 限制最大速度，防止避障力过猛
    vx = satfunc(vx, 0.4f);
    vy = satfunc(vy, 0.4f);
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
    nh.param<float>("err_max", err_max, 0.1);
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

    setpoint_raw.type_mask = 960;
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

        mission_pos_cruise_avoid(0, 0, ALTITUDE, 0, err_max);
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

        switch (mission_num)
        {
            case 1:
                ROS_INFO_ONCE("任务1: 起飞悬停");
                if (mission_pos_cruise_avoid(0, 0, ALTITUDE, 0, err_max)) {
                    mission_num = 2;
                    last_request = ros::Time::now();
                }
                break;

            case 2:
                ROS_INFO_ONCE("任务2: 记忆起始点颜色...");
                if (call_remember_color() && ros::Time::now() - last_request > ros::Duration(8.0) ) {
                    mission_num = 3;
                    last_request = ros::Time::now();
                } else if (ros::Time::now() - last_request >= ros::Duration(15.0)) {
                    mission_num = 3;
                    last_request = ros::Time::now();
                }
                mission_pos_cruise_avoid(0, 0, ALTITUDE, 0, err_max);
                break;
            case 3:
                ROS_INFO_ONCE("任务3:go to (2.5, 2.0)");
                if (mission_pos_cruise_avoid(2.5, 2.0, ALTITUDE, 0.0, err_max)) {
                    mission_num = 4;
                    last_request = ros::Time::now();
                }
                break;
	    case 4:
                ROS_INFO_ONCE("任务4:go to (5.0, -0.4)");
                if (mission_pos_cruise_avoid(5.0, -0.4, ALTITUDE, 0.0, err_max)) {
                    mission_num = 5;
                    last_request = ros::Time::now();
                }
                break;
            case 5:
                ROS_INFO_ONCE("任务5:go to (10.5, -0.4)");
                if (mission_pos_cruise_avoid(10.5, -0.4, ALTITUDE, 0.0, err_max)) {
                    mission_num = 51;
                    last_request = ros::Time::now();
                }
                break;
            case 51:
    		ROS_INFO_ONCE("任务51: 在门前悬停3秒");
    		mission_pos_cruise_avoid(10.5, -0.4, ALTITUDE, 0.0, err_max);
    		if (ros::Time::now() - last_request > ros::Duration(3.0)) {
        		mission_num = 6;
    		}
    		break;

            case 6:
                ROS_INFO_ONCE("任务6: 穿第一个通道 (17.0, -0.4)");
                if (mission_pos_cruise_avoid(17.0, -0.4, ALTITUDE, 0.0, err_max)) {
                    mission_num = 7;
                    last_request = ros::Time::now();
                }
                break;

            case 7:
                ROS_INFO_ONCE("任务7: go to (18.0, 2.4)");
                if (mission_pos_cruise_avoid(18.0, 2.4, ALTITUDE, 0.0, err_max)) {
                    mission_num = 71;
                    last_request = ros::Time::now();
                }
                break;
            case 71:
    		ROS_INFO_ONCE("任务71: 在门前悬停3秒");
    		mission_pos_cruise_avoid(18.0, 2.4, ALTITUDE, 0.0, err_max);
    		if (ros::Time::now() - last_request > ros::Duration(3.0)) {
        		mission_num = 8; 
    		}
    		break;
	    case 8:
                ROS_INFO_ONCE("任务8: 穿第一个通道 (20.5, 2.4)");
                if (mission_pos_cruise_avoid(20.5, 2.4, ALTITUDE, 0.0, err_max)) {
                    ROS_INFO("成功穿门！");
                    mission_num = 9;
                    last_request = ros::Time::now();
                }
                break;
                
            case 9:
                ROS_INFO_ONCE("任务9: go to (20.5, 0.2)");
                if (mission_pos_cruise_avoid(20.5, 0.2, ALTITUDE, 0.0, err_max)) {
                    mission_num = 91;
                    last_request = ros::Time::now();
                }
                break;
            case 91:
    		ROS_INFO_ONCE("任务91: 在门前悬停3秒");
    		mission_pos_cruise_avoid(21.0, 0.2, ALTITUDE, 0.0, err_max);
    		if (ros::Time::now() - last_request > ros::Duration(3.0)) {
        		mission_num = 10; // 然后穿门
    		}
    		break;    
            case 10:
                ROS_INFO_ONCE("任务10: 穿矩形门 (23.0, 0.2)");
                if (mission_pos_cruise_avoid(23.0, 0.2, ALTITUDE, 0.0, err_max)) {
                    ROS_INFO("成功穿门！");
                    mission_num = 11;
                    last_request = ros::Time::now();
                }
                break;
            case 11:
		{
    		ROS_INFO_ONCE("任务11: 飞往目标区域 (35, 0)");
    			if (mission_pos_cruise_avoid(35.0, 0.0, ALTITUDE, 0.0, err_max)) {
        		mission_num = 12;
        		last_request = ros::Time::now(); // 用于 case 12 的识别超时
    			}
		}
		break;

	     case 12:
		{
    		ROS_INFO_ONCE("任务12: 在 (35, 0) 悬停并识别降落点...");
    		// 保持当前位置悬停，防止漂移
    		mission_pos_cruise_avoid(35.0, 0.0, ALTITUDE, 0.0, err_max);

    		int target_id = call_find_target();
    		if (target_id > 0) {
       		target_color_id = target_id;
        		ROS_INFO("✅ 识别到颜色 %d 降落点", target_id);
        		mission_num = 13;
    		} else if (ros::Time::now() - last_request >= ros::Duration(8.0)) {
        		ROS_WARN("识别超时（8秒），进入搜索模式");
        		mission_num = 14;
        		search_step = 0;
    			}
		}
               break;

	     case 13:
		{
    		// 安全检查：若目标无效，使用安全点
    		if (target_color_id <= 0 || !target_points.count(target_color_id)) {
        	ROS_ERROR("目标点无效 (id=%d)，使用安全降落点", target_color_id);
       	target_points[-1] = {SAFE_LAND_X, SAFE_LAND_Y};
        	target_color_id = -1;
    		}

    		auto point = target_points[target_color_id];
    		ROS_INFO_ONCE("任务13: 飞往 %s降落点 (%.1f, %.1f)",
                  		(target_color_id == -1 ? "安全" : ("颜色" + 			  std::to_string(target_color_id)).c_str()),
                  point[0], point[1]);

               static bool arrived_at_land_point = false;
    		if (!arrived_at_land_point) {
        		if (mission_pos_cruise_avoid(point[0], point[1], ALTITUDE, 0.0, err_max)) {
            arrived_at_land_point = true;
            	last_request = ros::Time::now(); // 开始悬停计时
        		}
    		} else {
        // 悬停 1 秒确保稳定
        	mission_pos_cruise_avoid(point[0], point[1], ALTITUDE, 0.0, err_max);
        	if (ros::Time::now() - last_request > ros::Duration(1.0)) {
            		mission_num = 15;
            		arrived_at_land_point = false;
            		landing_state = 0; // 重置降落状态机
        		}
    		}
	}
	break;

	case 14:
	{
    		ROS_INFO_ONCE("任务14: 搜索模式 - 步骤 %d/3", search_step + 1);
    		if (search_step < 3) {
        	// 直接飞往三个真实色块位置（提高识别成功率）
        	float search_points[3][2] = {{35, 3}, {35, 1}, {35, -1}}; // 根据你的实际布局

        	if (mission_pos_cruise_avoid(search_points[search_step][0],
                                     search_points[search_step][1],
                                     ALTITUDE, 0.0, err_max)) {
            		int target_id = call_find_target();
            		if (target_id > 0) {
                		target_color_id = target_id;
                		ROS_INFO("🔍 在 (%.1f, %.1f) 识别到颜色 %d", 
                         		search_points[search_step][0], 
                         		search_points[search_step][1], 
                         		target_id);
                		mission_num = 13;
                		search_step = 0;
            		} else {
                		ROS_WARN("在 (%.1f, %.1f) 未识别到目标", 
                         	search_points[search_step][0], 
                         	search_points[search_step][1]);
                		search_step++;
                		last_request = ros::Time::now();
            			}
        		}
    		} else {
       		 ROS_ERROR("所有色块位置均未识别到目标，使用安全降落点 (%.1f, %.1f)", 
                  	SAFE_LAND_X, SAFE_LAND_Y);
        		target_points[-1] = {SAFE_LAND_X, SAFE_LAND_Y};
        		target_color_id = -1;
        		mission_num = 13;
        		search_step = 0;
    		}
	}
	break;

	case 15:
	{
    		ROS_INFO_ONCE("任务15: 启动精准平稳降落流程");
    		if (precision_land()) {
        	mission_num = -1; // 任务成功完成
    		}
	}
	break;    
	     
        }

        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
