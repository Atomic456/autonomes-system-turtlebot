#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
import math

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class TargetDetection(Node):
    def __init__(self):
        super().__init__("target_detection")
        self.target_steering_pub = self.create_publisher(Twist, '/target_cmd_vel', 10)
        self.target_mask_pub = self.create_publisher(Image, '/target_mask', 10)
        self.target_canny_pub = self.create_publisher(Image, '/target_canny', 10)
        self.create_subscription(Image, "/image_raw", self.detect_target, 10)
        self.cv_bridge = CvBridge()
        self.img_hight = 480
        self.img_width = 640
        self.get_logger().info("Target Detection Node created successfully!")
    
    def detect_target(self, img:Image):
        cv_img =  self.cv_bridge.imgmsg_to_cv2(img)
        self.img_hight, self.img_width, _ = cv_img.shape
        self.robot_dir = [0, self.img_hight * (-1)]
        blured_img = cv2.GaussianBlur(cv_img, (3,3), 0)
        hsv_img = cv2.cvtColor(blured_img, cv2.COLOR_BGR2HSV)
        target_img = self.mask_image(hsv_img)
        lines = self.houghLines(target_img)
        avrage_slope = self.avarge_hough_lines(lines)
        angle = self.calculate_obstacle_dir(avrage_slope)

        self.target_steering_pub.publish(self.convert_angel_to_steering(angle))


    def houghLines(self, masked_Image):
        target_edges = cv2.Canny(masked_Image, 252, 255)

        self.target_canny_pub.publish(self.cv_bridge.cv2_to_imgmsg(target_edges))

        min_line_length = 27
        max_line_gap = 2
        rho = 2
        theta = np.pi / 180
        hough_threshold = 13
        lines = cv2.HoughLinesP(target_edges, rho, theta, hough_threshold, np.array([]), minLineLength = min_line_length, maxLineGap = max_line_gap)
        return lines
    
    def mask_image(self, hsv_img):
        # Calculate the width of each sliver
        fourth = hsv_img.shape[1] // 4
        
        # Seperate image into three segments
        img_mask = hsv_img[:, fourth:3*fourth]

        # red
        red_lower, red_upper = self.get_limits([0,0,255])
        target_img_lower = cv2.inRange(img_mask, red_lower, red_upper)
        target_img_upper = cv2.inRange(img_mask, np.array([170, 120, 120]), np.array([180, 255, 255]))

        target_img = cv2.bitwise_or(target_img_lower, target_img_upper)

        self.target_mask_pub.publish(self.cv_bridge.cv2_to_imgmsg(target_img))

        return target_img

    def avarge_hough_lines(self, lines):
        if lines is None or len(line)==0:
            return 0

        slopes = []
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4)

            if x1 == x2:
                slope = float(99999) # not able to calculate slope
            elif x1 < x2:
               slope = float((y2 - y1)) / float((x2 - x1))
               slopes.append(slope)
            else:
                slope = float((y1 - y2)) / float((x1 - x2))
                slopes.append(slope)
        
        return np.average(slopes)
        
    
    def calculate_obstacle_dir(self, avrage_slope):
        if avrage_slope == 0:
            return 90
        
        self.target_dir = [1,avrage_slope]

        self.steering_dir = (avrage_slope / abs(avrage_slope)) * (-1)

        return np.arccos((self.robot_dir[0]*self.target_dir[0]+self.robot_dir[0]*self.target_dir[0])/(math.sqrt(self.robot_dir[0]**2+self.robot_dir[1]**2)*math.sqrt(self.target_dir[0]**2+self.target_dir[1]**2)))


    def convert_angel_to_steering(self, angle):
        angle = 90 - angle
        steering_val = (angle / 180) * np.pi * self.steering_dir
    
        twist_msg = Twist()
        twist_msg.angular.z = steering_val
        twist_msg.linear.x = 0.2
        return twist_msg
    
    def get_limits(self, color):
        c = np.uint8([[color]])  # BGR values
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue = hsvC[0][0][0]  # Get the hue value

        # Handle red hue wrap-around
        if hue >= 165:  # Upper limit for divided red hue
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([180, 255, 255], dtype=np.uint8)
        elif hue <= 15:  # Lower limit for divided red hue
            lowerLimit = np.array([0, 80, 80], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

        return lowerLimit, upperLimit

def main(args=None):
    rclpy.init(args=args)
    node = TargetDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
