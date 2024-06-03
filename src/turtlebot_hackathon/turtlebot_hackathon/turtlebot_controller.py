#!/usr/bin/env python3
import rclpy
import time

from rclpy.node import Node
from enum import Enum
from geometry_msgs.msg import Twist


class States(Enum):
    DEFAULT = 0
    INIT = 1
    EXPLORE = 2
    OBSTACLE = 3
    TARGET = 4



class TurtlebotController(Node):

    def __init__(self):
        super().__init__('controller_note')
        self.state = States.INIT
        self.steering_pub = self.create_publisher(Twist, '/valid_cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self.publish_steering, 10)
        self.create_subscription(Twist, '/obstacle_cmd_vel', self.obstacle_detected, 10)
        self.create_subscription(Twist, '/target_cmd_vel', self.target_detected, 10)
        self.initialize()

    def publish_steering(self, steering:Twist):
        if self.state is States.EXPLORE:
            self.steering_pub.publish(steering)
        
        if self.state is States.OBSTACLE:
            print('Obstacle detected')
            # only allow obstacle avoidance
            # disable path execution
            # use timer to change back into explore mode

        if self.state is States.TARGET:
            print('target detected')
            #disable task excecution
            #drive towords target

    def obstacle_detected(self, steering:Twist):
        self.state = States.OBSTACLE
        self.steering_pub.publish(steering)
        time.sleep(3)
        self.state = States.EXPLORE

    
    def target_detected(self, steering:Twist):
        self.state = States.TARGET
        self.steering_pub.publish(steering)


    def initialize(self):
        if self.state is States.INIT:
            rotation = 0
            while round < 360:
                self.shuffle_position()
                self.reorientation(rotation)
            self.state = States.EXPLORE

    def shuffle_position(self):
        twist_msg = Twist()
        twist_msg.z = 0.0
        twist_msg.x = 0.1
        self.steering_pub.publish(twist_msg)
        time.sleep(2)
        twist_msg.x = -0.1
        self.steering_pub.publish(twist_msg)
        time.sleep(2)

    def reorientation(self, angle):
        #rotate the robot 306° around its center axis
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 1.57  # 90 degrees in radians
        angle = angle + 90
        self.steering_pub.publish(twist_msg)
        time.sleep(1)
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()