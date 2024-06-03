#!/usr/bin/env python3
import rclpy

from rclpy.node import Node


class MapExploration(Node):

    def __init__(self):
        super().__init__('explor_note')


def main(args=None):
    rclpy.init(args=args)
    node = MapExploration()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()