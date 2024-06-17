#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance 
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


class ObjectAvoidance(Node):

    def __init__(self):
        super().__init__("oav2_node")
        self.ack_pos = None
        self.behavior = 0
        self.movement_pub = self.create_publisher(Twist, '/valid_cmd_vel', 10)
        self.img_l_pub = self.create_publisher(Image, '/image_left', 10)
        self.img_m_pub = self.create_publisher(Image, '/image_middle', 10)
        self.img_r_pub = self.create_publisher(Image, '/image_right', 10)
        self.behavior_pub = self.create_publisher(Int8, '/behavior', 10)
        self.goal_pos_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(Int8, '/behavior', self.update_behavior, 10)
        self.create_subscription(Twist, '/cmd_vel', self.validate_steering, 10)
        self.create_subscription(PoseStamped, '/pose', self.update_pos, 10)
        self.create_subscription(Image, "/image_raw", self.local_path_planning, 10)
        self.cv_bridge = CvBridge()
        self.scale = 4
        self.get_logger().info("ObjectAvoidance created successfully!")

    def validate_steering(self, steering:Twist):
        if self.behavior == 2:
            self.movement_pub.publish(steering)
        if self.behavior == 4:
            self.goal_pos_pub.publish(self.ack_pos)

    
    def update_behavior(self, behavior: Int8):
        self.behavior = behavior
    

    def update_pos(self, pos:PoseStamped):
        self.ack_pos = pos

    def local_path_planning(self, img:Image):
        self.get_logger().info("Starting path planning...")
        pi_cam_img = self.cv_bridge.imgmsg_to_cv2(img)
        obstacle_img, target_img = self.image_pre_processing(pi_cam_img)
        self.swich_behavior(obstacle_img,target_img)

        if self.behavior == 3:
            hue_left, hue_middle, hue_right = self.mask_image(obstacle_img)
            self.calculate_movement(hue_left=hue_left, hue_middle=hue_middle, hue_right=hue_right)
        elif self.behavior == 4:
            hue_left, hue_middle, hue_right = self.mask_image(target_img)
            self.calculate_movement(hue_left=hue_left, hue_middle=hue_middle, hue_right=hue_right)

    def image_pre_processing(self, img):
        #self.get_logger().info("Pre processing image...")
        #bluring
        blured_img = cv2.GaussianBlur(img, (5,5), 0)
        #cv2.imwrite(self.save_img_path+"blured"+str(self.counter)+".jpg", blured_img)
        hsv_image = cv2.cvtColor(blured_img, cv2.COLOR_BGR2HSV)

        #color masking
        red = [0,0,255]
        lowerLimit, upperLimit = self.get_limits(red)
        print(lowerLimit, upperLimit)
        red_color_mask = cv2.inRange(hsv_image, lowerLimit, upperLimit)

        yellow_color_mask = cv2.inRange(hsv_image, np.array([40, 60, 32]), np.array([60, 100, 100]))

        return yellow_color_mask, red_color_mask
    
    def mask_image(self, obstacel_img):
        # Calculate the width of each sliver
        width = obstacel_img.shape[1] // 3

        # Seperate image into three segments
        img_mask_right = obstacel_img[:, :width]
        self.img_r_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_mask_right))
        img_mask_middle = obstacel_img[:, width:2*width]
        self.img_m_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_mask_middle))
        img_mask_left = obstacel_img[:, 2*width:]
        self.img_l_pub.publish(self.cv_bridge.cv2_to_imgmsg(img_mask_left))

        # Calculate intensity of obstacles
        hue_right = np.mean(img_mask_right)
        hue_middle = np.mean(img_mask_middle)
        hue_left = np.mean(img_mask_left)

        return hue_left, hue_middle, hue_right
    
    def calculate_movement(self, hue_left, hue_middle, hue_right):
        twist_msg = Twist()
        least_obstacles = min([hue_left, hue_middle, hue_right])

        if least_obstacles == hue_left:
            self.get_logger().info("Hue left: " + str(least_obstacles))
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = (least_obstacles / 255) * (1.5) * self.scale
        elif least_obstacles == hue_middle:
            self.get_logger().info("Hue middle: " + str(least_obstacles))
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 0.0
        else:
            self.get_logger().info("Hue right: " + str(least_obstacles))
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = (least_obstacles / 255) * (-1.5) * self.scale

        self.movement_pub.publish(twist_msg)
        

    def swich_behavior(self, obstacle_img, target_img):
        img_max_fill = 255
        t_hue_left, t_hue_center, t_hue_right = self.mask_image(target_img)
        o_hue_left, o_hue_center, o_hue_right = self.mask_image(obstacle_img)

        if o_hue_center > img_max_fill*(1/6):
            bhv_msg = Int8()
            bhv_msg.data = 3
            self.behavior_pub.publish(bhv_msg)
        elif (t_hue_left+t_hue_center+t_hue_right) > img_max_fill*(1/20):
            bhv_msg = Int8()
            bhv_msg.data = 4
            self.behavior_pub.publish(bhv_msg)
        



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
