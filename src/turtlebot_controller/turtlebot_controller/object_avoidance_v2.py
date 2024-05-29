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
        super().__init__("oav2_node")
        self.create_subscription(Image, "/image_raw", self.local_path_planning, 10)
        self.cv_bridge = CvBridge()
        self.save_img_path = "/home/ubuntu/ros2_ws/images/modified/"
        self.counter = 0
        self.get_logger().info("ObjectAvoidance created successfully!")

    def local_path_planning(self, img:Image):
        self.get_logger().info("Starting path planning...")
        pi_cam_img = self.cv_bridge.imgmsg_to_cv2(img)
        self.image_pre_processing(pi_cam_img)

    def image_pre_processing(self, img):
        self.get_logger().info("Pre processing image...")
        #bluring
        blured_img = cv2.GaussianBlur(img, (5,5), 0)
        cv2.imwrite(self.save_img_path+"blured"+str(self.counter)+".jpg", blured_img)

        #color masking
        red = [0,0,255]
        lowerLimit, upperLimit = self.get_limits(red)
        red_color_mask = cv2.inRange(blured_img, lowerLimit, upperLimit)
        blue = [255,0,0]
        lowerLimit, upperLimit = self.get_limits(blue)
        blue_color_mask = cv2.inRange(blured_img, lowerLimit, upperLimit)
        cv2.imwrite(self.save_img_path+"blue"+str(self.counter)+".jpg", blue_color_mask)
        yellow = [0,255,255]
        lowerLimit, upperLimit = self.get_limits(yellow)
        yellow_color_mask = cv2.inRange(blured_img, lowerLimit, upperLimit)
        cv2.imwrite(self.save_img_path+"yellow"+str(self.counter)+".jpg", yellow_color_mask)
        white = [255,255,255]
        lowerLimit, upperLimit = self.get_limits(white)
        white_color_mask = cv2.inRange(blured_img, lowerLimit, upperLimit)
        cv2.imwrite(self.save_img_path+"white"+str(self.counter)+".jpg", white_color_mask)

        #recombine image masks
        combined_obstacel_color_mask = cv2.bitwise_or(blue_color_mask,yellow_color_mask)
        combined_obstacel_color_mask = cv2.bitwise_or(combined_obstacel_color_mask,white_color_mask)

        """ Debug output """
        self.get_logger().info("Saving solution for Image no." + str(self.counter) + " add path " + self.save_img_path+"obstacle"+str(self.counter)+".jpg")
        cv2.imwrite(self.save_img_path+"obstacle"+str(self.counter)+".jpg", combined_obstacel_color_mask)
        cv2.imwrite(self.save_img_path+"red"+str(self.counter)+".jpg", red_color_mask)
        self.counter = self.counter + 1

        return combined_obstacel_color_mask, red_color_mask
    

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
    node = ObjectAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()