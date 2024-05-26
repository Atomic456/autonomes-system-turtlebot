#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np
import math

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class ObjectAvoidance(Node):

    def __init__(self):
        super().__init__("objavo_node")
        #self.create_subscription(Image, "/image_raw", self.preprocess_image, 10)
        self.counter = 0
        self.create_subscription(Image, "/image_raw", self.color_mask_img, 10)
        self.cv_bridge = CvBridge()
        self.get_logger().info("ObjectAvoidance created successfully!")

    def preprocess_image(self, img:Image):
        pi_cam_img = cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(img), cv2.COLOR_BGR2GRAY)
        self.img_hight, self.img_width = pi_cam_img.shape

        img_blured = cv2.GaussianBlur(pi_cam_img, (4,4), 0)

        canny_img = cv2.Canny(img_blured, 200,255)

        lines = self.detect_lines(canny_img=canny_img)

    def detect_lines(self, canny_img):
        min_line_length = 27
        max_line_gap = 2
        rho = 2
        theta = np.pi /180
        hough_threshold = 13
        lines = cv2.HoughLinesP(canny_img, rho, theta, hough_threshold, np.array([]), minLineLength = min_line_length, maxLineGap = max_line_gap)
        self.get_logger().info("============ HoughLines ============")
        self.get_logger().info(lines)
        self.get_logger().info("====================================")
        return lines
    
    def calculate_approach_angle(self, detected_lines):
        angles = []
        if detected_lines is None:
            return angles
        for line in detected_lines:
            x1,y1,x2,y2 = [0,0,0,0]
            x1,y1,x2,y2 = line.reshape(4)

            approach_angle =  np.arccos(((x1)*(x2))+((y1)*(y2))/(math.sqrt((x1**2)+(y1**2)))*(math.sqrt((x2**2)+(y2**2))))
            angles.append(approach_angle)
        
        return angles#
    
    def color_mask_img(self, img:Image):
        save_img_path = "/home/ubuntu/ros2_ws/images/modified/"

        img = self.cv_bridge.imgmsg_to_cv2(img)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_limit, upper_limit = self.get_limits(color=[0, 0, 255])
        red_img = cv2.inRange(hsv_img, lower_limit, upper_limit)
        cv2.imwrite(save_img_path+"red_img"+str(self.counter)+".jpg", red_img)
        self.counter = self.counter+1

        lower_limit, upper_limit = self.get_limits(color=[255, 0, 0])
        blue_img = cv2.inRange(hsv_img, lower_limit, upper_limit)
        cv2.imwrite(save_img_path+"blue_img"+str(self.counter)+".jpg", blue_img)
        self.counter = self.counter+1

        lower_limit, upper_limit = self.get_limits(color=[0, 255, 255])
        yellow_img = cv2.inRange(hsv_img, lower_limit, upper_limit)
        cv2.imwrite(save_img_path+"yellow_img"+str(self.counter)+".jpg", yellow_img)
        self.counter = self.counter+1

        return red_img, yellow_img, blue_img
    
    def get_limits(self, color):
        c = np.uint8([[color]])  # BGR values
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue = hsvC[0][0][0]  # Get the hue value

        # Handle red hue wrap-around
        if hue >= 165:  # Upper limit for divided red hue
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([180, 255, 255], dtype=np.uint8)
        elif hue <= 15:  # Lower limit for divided red hue
            lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
        else:
            lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

        return lowerLimit, upperLimit


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()