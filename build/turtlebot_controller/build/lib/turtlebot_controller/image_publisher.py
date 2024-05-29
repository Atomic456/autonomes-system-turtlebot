import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("img_pub")
        self.counter = 0
        self.img_pub = self.create_publisher(Image, "/image_raw", 10)
        self.timer = self.create_timer(10,self.publish_image)
        self.cv_bridge = CvBridge()
        self.get_logger().info("ImagePublisher created successfully!")

    def publish_image(self):
        self.get_logger().info("/home/ubuntu/ros2_ws/images/"+"IMG_"+str(self.counter+1)+".jpg")
        img = cv2.imread(cv2.samples.findFile("/home/ubuntu/ros2_ws/images/"+"IMG_"+str(self.counter+1)+".jpg"))
        cv2.imshow("Display window", img)
        img_msg = self.cv_bridge.cv2_to_imgmsg(img)
        self.counter = (self.counter + 1)%14
        self.img_pub.publish(img_msg)

    


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()