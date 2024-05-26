#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image



class ObjectAvoidance(Node):

    def __init__(self):
        super().__init__("objavo_node")
        self.get_logger().info("Contructor run successfully!")
        self.create_subscription(Image, "/image_raw", self.preprocess_image, 10)

    def preprocess_image(self, img:Image):
        string = ""


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()