#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from color_landing.srv import RememberColor, FindTarget, GetColor

class ColorMemory:
    def __init__(self):
        self.remembered_color = None
        self.bridge = CvBridge()
        
        # 颜色阈值 (根据你的仿真环境调整)
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'yellow': ([20, 100, 100], [30, 255, 255]),
            'blue': ([100, 150, 50], [130, 255, 255])
        }
        
        # ROS服务
        rospy.Service('/color_landing/remember_color', RememberColor, self.remember_color_cb)
        rospy.Service('/color_landing/find_target', FindTarget, self.find_target_cb)
        rospy.Service('/color_landing/get_color', GetColor, self.get_color_cb)
        
        # 订阅摄像头 (根据你的仿真调整topic)
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.image_callback)
        self.current_frame = None
        
        rospy.loginfo("颜色记忆节点已初始化")
    
    def image_callback(self, msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logwarn("图像接收失败: %s", e)
    
    def remember_color_cb(self, req):
        if self.current_frame is None:
            return False, "无图像数据"
            
        color = self.detect_dominant_color(self.current_frame)
        if color:
            self.remembered_color = color
            rospy.loginfo("已记忆颜色: %s", color)
            return True, "记忆成功: " + color
        return False, "未识别到颜色"
    
    def find_target_cb(self, req):
        if not self.remembered_color:
            return 0, "请先记忆颜色"
            
        if self.current_frame is None:
            return 0, "无图像数据"
            
        current_color = self.detect_dominant_color(self.current_frame)
        if current_color == self.remembered_color:
            color_id = self.color_to_id(current_color)
            return color_id, "找到目标: " + current_color
        return 0, "未找到匹配颜色"
    
    def get_color_cb(self, req):
        return self.color_to_id(self.remembered_color) if self.remembered_color else 0
    
    def detect_dominant_color(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        max_pixels = 0
        dominant_color = None
        
        for color_name, (lower, upper) in self.color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
            pixel_count = cv2.countNonZero(mask)
            
            if pixel_count > max_pixels and pixel_count > 500:  # 阈值可调
                max_pixels = pixel_count
                dominant_color = color_name
                
        return dominant_color
    
    def color_to_id(self, color_name):
        color_map = {'red': 1, 'yellow': 2, 'blue': 3}
        return color_map.get(color_name, 0)

if __name__ == "__main__":
    rospy.init_node('color_memory')
    ColorMemory()
    rospy.spin()
