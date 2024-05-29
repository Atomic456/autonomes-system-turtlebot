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
        combined_obstacel_color_mask, red_color_mask = self.image_pre_processing(pi_cam_img)
        hue_left, hue_middle, hue_right = self.mask_image(combined_obstacel_color_mask)
        self.get_logger().info("hue_left: " + str(hue_left) +" | hue_middle: " + str(hue_middle) + " | hue_left: " + str(hue_right))
        steer = self.calculate_movement(hue_left=hue_left, hue_middle=hue_middle, hue_right=hue_right)
        self.get_logger().info("Steering move: " + steer)

    def image_pre_processing(self, img):
        self.get_logger().info("Pre processing image...")
        #bluring
        blured_img = cv2.GaussianBlur(img, (5,5), 0)
        #cv2.imwrite(self.save_img_path+"blured"+str(self.counter)+".jpg", blured_img)
        hsv_image = cv2.cvtColor(blured_img, cv2.COLOR_BGR2HSV)

        #color masking
        red = [0,0,255]
        lowerLimit, upperLimit = self.get_limits(red)
        red_color_mask = cv2.inRange(hsv_image, lowerLimit, upperLimit)
        blue = [255,0,0]
        lowerLimit, upperLimit = self.get_limits(blue)
        blue_color_mask = cv2.inRange(hsv_image, lowerLimit, upperLimit)
        #cv2.imwrite(self.save_img_path+"blue"+str(self.counter)+".jpg", blue_color_mask)
        yellow = [0,255,255]
        lowerLimit, upperLimit = self.get_limits(yellow)
        yellow_color_mask = cv2.inRange(hsv_image, lowerLimit, upperLimit)
        #cv2.imwrite(self.save_img_path+"yellow"+str(self.counter)+".jpg", yellow_color_mask)
        white = [255,255,255]
        lowerLimit, upperLimit = self.get_limits(white)
        white_color_mask = cv2.inRange(hsv_image, lowerLimit,  upperLimit)
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
    
    def mask_image(self, obstacel_img):

        # Calculate the width of each sliver
        width = obstacel_img.shape[1] // 3

        #base_img = np.zeros_like(obstacel_img)
        #img_mask_right = cv2.fillPoly(base_img, mask_right, 255)
        # Extract the three slivers
        img_mask_right = obstacel_img[:, :width]
        img_mask_middle = obstacel_img[:, width:2*width]
        img_mask_left = obstacel_img[:, 2*width:]
        
        ''' Debug output '''
        cv2.imwrite(self.save_img_path+"img_mask_right"+str(self.counter)+".jpg", img_mask_right)
        cv2.imwrite(self.save_img_path+"img_mask_middle"+str(self.counter)+".jpg", img_mask_middle)
        cv2.imwrite(self.save_img_path+"img_mask_left"+str(self.counter)+".jpg", img_mask_left)

        hue_right = np.mean(img_mask_right)
        hue_middle = np.mean(img_mask_middle)
        hue_left = np.mean(img_mask_left)

        return hue_left, hue_middle, hue_right
    
    def calculate_movement(self, hue_left, hue_middle, hue_right):
        least_obstacles = min([hue_left, hue_middle, hue_right])

        if least_obstacles == hue_left:
            steer = "steer left"
        elif least_obstacles == hue_middle:
            steer = "reset steering" 
        else:
            steer = "steer right"
        
        return steer






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