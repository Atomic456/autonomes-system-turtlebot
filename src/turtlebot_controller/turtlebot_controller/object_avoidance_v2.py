#!/usr/bin/env python3
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from enum import Enum
from geometry_msgs.msg import Twist


class Behavior(Enum):
    REORIENT = 1
    AVOID_OBSTACLES = 2
    TARGET_LOCK = 3


class ObjectAvoidance(Node):

    def __init__(self):
        super().__init__("oav2_node")
        self.create_subscription(Image, "/image_raw", self.local_path_planning, 10)
        self.movement_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cv_bridge = CvBridge()
        self.save_img_path = "/home/ubuntu/ros2_ws/images/modified/"
        self.counter = 0
        self.behavior = Behavior.AVOID_OBSTACLES
        self.get_logger().info("ObjectAvoidance created successfully!")

    def local_path_planning(self, img:Image):
        self.get_logger().info("Starting path planning...")
        pi_cam_img = self.cv_bridge.imgmsg_to_cv2(img)
        obstacle_img, target_img = self.image_pre_processing(pi_cam_img)
        self.swich_behavior(obstacle_img,target_img)

        if self.behavior is Behavior.AVOID_OBSTACLES:
            hue_left, hue_middle, hue_right = self.mask_image(obstacle_img)
            self.calculate_movement(hue_left=hue_left, hue_middle=hue_middle, hue_right=hue_right)
        elif self.behavior is Behavior.TARGET_LOCK:
            hue_left, hue_middle, hue_right = self.mask_image(target_img)
            self.calculate_movement(hue_left=hue_left, hue_middle=hue_middle, hue_right=hue_right)
        else:
            #turn 90° and reassess
            self.ninety_degree_turn()

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
        #cv2.imwrite(self.save_img_path+"white"+str(self.counter)+".jpg", white_color_mask)

        #recombine image masks
        combined_obstacel_color_mask = cv2.bitwise_or(blue_color_mask,yellow_color_mask)
        combined_obstacel_color_mask = cv2.bitwise_or(combined_obstacel_color_mask,white_color_mask)

        """ Debug output """
        #self.get_logger().info("Saving solution for Image no." + str(self.counter) + " add path " + self.save_img_path+"obstacle"+str(self.counter)+".jpg")
        #cv2.imwrite(self.save_img_path+"obstacle"+str(self.counter)+".jpg", combined_obstacel_color_mask)
        #cv2.imwrite(self.save_img_path+"red"+str(self.counter)+".jpg", red_color_mask)
        #self.counter = self.counter + 1

        return combined_obstacel_color_mask, red_color_mask
    
    def mask_image(self, obstacel_img):
        # Calculate the width of each sliver
        width = obstacel_img.shape[1] // 3

        # Seperate image into three segments
        img_mask_right = obstacel_img[:, :width]
        img_mask_middle = obstacel_img[:, width:2*width]
        img_mask_left = obstacel_img[:, 2*width:]

        ''' Debug output '''
        #cv2.imwrite(self.save_img_path+"img_mask_right"+str(self.counter)+".jpg", img_mask_right)
        #cv2.imwrite(self.save_img_path+"img_mask_middle"+str(self.counter)+".jpg", img_mask_middle)
        #cv2.imwrite(self.save_img_path+"img_mask_left"+str(self.counter)+".jpg", img_mask_left)

        # Calculate intensity of obstacles
        hue_right = np.mean(img_mask_right)
        hue_middle = np.mean(img_mask_middle)
        hue_left = np.mean(img_mask_left)

        return hue_left, hue_middle, hue_right
    
    def calculate_movement(self, hue_left, hue_middle, hue_right):
        twist_msg = Twist()
        least_obstacles = min([hue_left, hue_middle, hue_right])

        if least_obstacles == hue_left:
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = least_obstacles / 255 * (1.5)
        elif least_obstacles == hue_middle:
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.0
        else:
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = least_obstacles / 255 * (-1.5)

        self.movement_pub.publish(twist_msg)
        

    def swich_behavior(self, obstacle_img, target_img):
        img_max_fill = 255
        target_img_intensity = np.mean(target_img)
        obstacle_img_intensity = np.mean(obstacle_img)

        if target_img_intensity > img_max_fill*(1/9):
            self.behavior = Behavior.TARGET_LOCK
        elif obstacle_img_intensity > img_max_fill*(4/5):
            self.behavior = Behavior.REORIENT
        else:
            self.behavior = Behavior.AVOID_OBSTACLES

    def ninety_degree_turn(self):
        twist_msg = Twist()
        twist_msg.angular.z = 1.57  # 90 degrees in radians
        self.movement_pub.publish(twist_msg)
        



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