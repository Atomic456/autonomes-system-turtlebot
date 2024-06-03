#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class ImageProcessing(Node):

    def __init__(self):
        super().__init__('img_processing_note')
        self.img_l_pub = self.create_publisher(Image, '/image_left', 10)
        self.img_m_pub = self.create_publisher(Image, '/image_middle', 10)
        self.img_r_pub = self.create_publisher(Image, '/image_right', 10)
        self.goal_pos_pub = self.create_publisher(PoseStamped, '/goal_pos', 10)
        self.obstacle_pub = self.create_publisher(Twist, '/obstacle_cmd_vel', 10)
        self.target_pub = self.create_publisher(Twist, '/target_cmd_vel', 10)
        
        self.create_subscription(Image, "/image_raw", self.process_image, 10)

        self.cv_bridge = CvBridge()
        self.scale = 4
        self.get_logger().info("Image processing created successfully!")

    def process_image(self, img:Image):
        self.get_logger().info("Process image...")
        pi_cam_img = self.cv_bridge.imgmsg_to_cv2(img)
        blured_img = cv2.GaussianBlur(pi_cam_img, (5,5), 0)
        hsv_img = cv2.cvtColor(blured_img, cv2.COLOR_BGR2HSV)
        obstacle_img, target_img = self.mask_image(hsv_img)
        o_left, o_center, o_right = self.devide_image(obstacle_img)
        t_left, t_center, t_right = self.devide_image(target_img)
        self.avoid_obstacle(obstacle_img, (1/9), o_left, o_center, o_right)
        self.find_traget(target_img, (1/12), t_left, t_center, t_right)


    def avoid_obstacle(self, masked_img, threshold, left_img, center_img, right_img):
        height, width = masked_img.shape[:2]

        detection_threshold = height*width*255*threshold

        if np.sum(masked_img) < detection_threshold:
            return
        
        # Calculate intensity of obstacles
        hue_middle = np.mean(left_img)
        hue_left = np.mean(center_img)
        hue_right = np.mean(right_img)
        free_path = min([hue_left,hue_middle,hue_right])

        steering = Twist()
        if free_path == hue_left:
            self.get_logger().info("Left path free!")
            steering.linear.x = 0.2
            steering.angular.z = (free_path / 255) * (1.5) * self.scale
        elif free_path == hue_right:
            self.get_logger().info("Right path free!")
            steering.linear.x = 0.2
            steering.angular.z = (free_path / 255) * (-1.5) * self.scale
        else:
            self.get_logger().info("Center path free!")
            steering.linear.x = 0.2
            steering.angular.z = 0.0
        
        self.obstacle_pub.publish(steering)



    def find_traget(self, masked_img, threshold, left_img, center_img, right_img):
        height, width = masked_img.shape[:2]

        detection_threshold = height*width*255*threshold

        if np.sum(masked_img) < detection_threshold:
            return
        
        # Calculate intensity of obstacles
        hue_middle = np.mean(left_img)
        hue_left = np.mean(center_img)
        hue_right = np.mean(right_img)
        traget_path = max([hue_left,hue_middle,hue_right])

        steering = Twist()
        if traget_path == hue_left:
            self.get_logger().info("Target in left path!")
            steering.linear.x = 0.2
            steering.angular.z = (traget_path / 255) * (1.5) * self.scale
        elif traget_path == hue_right:
            self.get_logger().info("Target in right path!")
            steering.linear.x = 0.2
            steering.angular.z = (traget_path / 255) * (-1.5) * self.scale
        else:
            self.get_logger().info("Target in center path!")
            steering.linear.x = 0.2
            steering.angular.z = 0.0
        
        self.target_pub.publish(steering)
    
    def mask_image(self, img):
        red_color_mask = cv2.inRange(img, np.array([315, 50, 20]), np.array([360, 100, 100]))
        yellow_color_mask = cv2.inRange(img, np.array([40, 60, 32]), np.array([60, 100, 100]))
        return yellow_color_mask, red_color_mask
    
    def devide_image(self, img):
        # Calculate the width of each sliver
        width = img.shape[1] // 3

        # Seperate image into three segments
        img_mask_right = img[:, :width]
        self.img_r_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_mask_right))
        img_mask_middle = img[:, width:2*width]
        self.img_m_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_mask_middle))
        img_mask_left = img[:, 2*width:]
        self.img_l_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_mask_left))


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessing()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()