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
        self.get_logger().info("Contructor run successfully!")
        self.create_subscription(Image, "/image_raw", self.preprocess_image, 10)
        self.cv_bridge = CvBridge()

    def preprocess_image(self, img:Image):
        pi_cam_img = cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(img), cv2.COLOR_BGR2GRAY)
        self.img_hight, self.img_width = pi_cam_img.shape

        img_blured = cv2.GaussianBlur(pi_cam_img, (4,4), 0)

        canny_img = cv2.Canny(img_blured, 200,255)

        lines = self.detect_lines(canny_img=canny_img)

    def detect_lines(self, canny_img:Image):
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
        
        return angles





def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()