#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__("object_avoidance_node")
        self.obstacle_pub = self.create_publisher(Twist, '/obstacle_cmd_vel', 10)
        self.target_pub = self.create_publisher(Twist, '/target_cmd_vel', 10)
        self.create_subscription(Image, "/image_raw", self.process_image, 10)
        self.get_logger().info("Obstacle Avoidance created successfully!")
        self.cv_bridge = CvBridge()
        self.img_hight = 480
        self.img_center = 320


    def process_image(self, img:Image):
        
        pi_cam_img  = self.cv_bridge.imgmsg_to_cv2(img)

        self.img_hight = pi_cam_img.shape[1]
        self.img_center = pi_cam_img.shape[0] // 2

        blured_img = cv2.GaussianBlur(pi_cam_img, (3,3), 0)
        hsv_img = cv2.cvtColor(blured_img, cv2.COLOR_BGR2HSV)
        target_mask = cv2.inRange(hsv_img, np.array([0, 80, 80]), np.array([15, 255, 255]))
        obstacle_img = cv2.inRange(hsv_img, np.array([40, 60, 32]), np.array([60, 100, 100]))

        target_edges = cv2.Canny(target_mask, 252, 255)
        obstacle_edges = cv2.Canny(obstacle_img, 252, 255)
        target_lines = self.houghLines(target_edges)
        obstacle_lines = self.houghLines(obstacle_edges)

        for line in range(target_lines):
            self.caluclate_angel(target_lines[line][0],target_lines[line][1],target_lines[line][2],target_lines[line][3])




    def caulate_angle(self, x1, y1, x2, y2):

        v1 = [0, -self.img_hight]
        v2 = [x1-x2, y1-y2]

        scalar = v1[0] * v2[0] + v1[1] * v2[1]
        v1_length = np.sqrt(v1[0]**2+v1[1]**2)
        v2_length = np.sqrt(v2[0]**2+v2[1]**2)

        intersect_angle = np.arccos(scalar / (v1_length * v2_length))
        print(intersect_angle)
        return intersect_angle


        


    def houghLines(self, masked_Image):
        min_line_length = 10
        max_line_gap = 2
        rho = 2
        theta = np.pi / 180
        hough_threshold = 8
        lines = cv2.HoughLinesP(masked_Image, rho, theta, hough_threshold, np.array([]), minLineLength = min_line_length, maxLineGap = max_line_gap)
        return lines
    
    def calculateLineSlope(self, lane_lines):
        slopes = []
        res_lines = []
        if lane_lines is None:
            return res_lines, slopes
        for line in lane_lines:
            x1, y1, x2, y2 = [0,0,0,0]
            x1, y1, x2, y2 = line.reshape(4)

            if x1 == x2:
                slope = float(99999) # not able to calculate slope
            elif x1 < x2:
               slope = float((y2 - y1)) / float((x2 - x1))
               slopes.append(slope)
               res_lines.append(line)
            else:
                slope = float((y1 - y2)) / float((x1 - x2))
                slopes.append(slope)
                res_lines.append(line)
        
        return res_lines, slopes


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
