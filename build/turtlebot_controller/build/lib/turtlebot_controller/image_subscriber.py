import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("img_sub")
        self.create_subscription(Image, "/image_raw", self.display_img, 10)
        self.cv_bridge = CvBridge()
        self.get_logger().info("ImageSubscriber created successfully!")

    def display_img(self, img:Image):
        cv2_img = self.cv_bridge.imgmsg_to_cv2(img)
        self.get_logger().info("imgshow...")
        cv2.imshow("Camera Image", cv2_img)

    


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()