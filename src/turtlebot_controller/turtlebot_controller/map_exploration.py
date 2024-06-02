import rclpy
import time
import random

from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from std_msgs.msg import PoseStamped 
from std_msgs.msg import OccupancyGrid
from enum import Enum


class Behavior(Enum):
    INIT = 1
    MAP_EXPLORATION = 2
    OBSTACLE_AVOIDANCE = 3
    TARGET_LOCK = 4

class MapExploration(Node):
    def __init__(self):
        super().__init__("nav_node")
        self.movement_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.movement_pub = self.create_publisher(PoseStamped, '/goal_pos', 10)
        self.behavior_pub = self.create_publisher(Int8, '/behavior', 10)
        self.create_subscription(Int8, '/behavior', self.update_behavior, 10)
        self.behavior_pub.publish(Int8(Behavior.INIT))
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', self.safe_map,10)
        self.location_initialization()
        self.create_timer(20,self.calculate_goal_pos)

        

    def safe_behavior(self, behavior:Int8):
        self.behavior = behavior

    def calculate_goal_pos(self):
        if self.behavior is Behavior.MAP_EXPLORATION:
            frontier = random.choice(self.find_frontiers())
            goal_pos = PoseStamped()
            goal_pos.header.frame_id = 'map'  # Set the reference frame (e.g., 'map')
            goal_pos.pose.position.x = frontier[0]  # Set the X-coordinate of the goal position
            goal_pos.pose.position.y = frontier[1] # Set the Y-coordinate of the goal position


    def safe_map(self, map:OccupancyGrid):
        self.map = map
        self.get_logger().info("Map updated!")

    def location_initialization(self):
        if self.behavior is Behavior.INIT:
            angle = 0
            while angle < 360:
                self.shuffle_position()
                angle = self.reorientation(angle)

    def shuffle_position(self):
        twist_msg = Twist()
        twist_msg.z = 0.0
        twist_msg.x = 0.1
        self.movement_pub.publish(twist_msg)
        time.sleep(2)
        twist_msg.x = -0.1
        self.movement_pub.publish(twist_msg)
        time.sleep(2)
    
    def reorientation(self, angle):
        #rotate the robot 306° around its center axis
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 1.57  # 90 degrees in radians
        angle = angle + 90
        self.movement_pub.publish(twist_msg)
        time.sleep(1)
        return angle
    
    def find_frontiers(self, map_data:OccupancyGrid):
        frontiers = []
        for x in range(len(map_data)):
            for y in range(len(map_data[0])):
                if self.is_frontier_cell(x, y, map_data):
                    frontiers.append((x, y))
        return frontiers

    def is_frontier_cell(self,x, y, map_data:OccupancyGrid):
        # Check if the cell is adjacent to both explored and unexplored cells
        if self.is_unknown_cell(x, y, map_data):
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if not self.is_unknown_cell(x + dx, y + dy, map_data):
                        return True
        return False

    def is_unknown_cell(x, y, map_data:OccupancyGrid):
        # Check if the cell is unexplored (unknown)
        return map_data[x][y] == -1


def main(args=None):
    rclpy.init(args=args)
    node = MapExploration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()