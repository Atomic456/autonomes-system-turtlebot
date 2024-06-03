#!/usr/bin/env python3
import rclpy
import random

from rclpy.node import Node
from std_msgs.msg import PoseStamped 
from std_msgs.msg import OccupancyGrid


class MapExploration(Node):
    def __init__(self):
        super().__init__('explor_note')
        self.goal_pos_pub = self.create_publisher(PoseStamped, '/goal_pos', 10)
        self.create_subscription(OccupancyGrid, '/map', self.safe_map,10)
        
        self.create_timer(10,self.calculate_goal_pos)
        self.get_logger().info("Map exploration started!")


    
    def calculate_goal_pos(self):
            goal_pos = PoseStamped()
            goal_pos.header.frame_id = 'map'  # Set the reference frame (e.g., 'map')
            goal_pos.pose.position.x = random.uniform(5.0, 5.9)  # Set the X-coordinate of the goal position
            goal_pos.pose.position.y = random.uniform(0.5, 2.5)
            self.goal_pos_pub.publish(goal_pos)



def main(args=None):
    rclpy.init(args=args)
    node = MapExploration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()