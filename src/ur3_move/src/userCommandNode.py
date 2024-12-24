#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import random
import time

class userCommandNode(Node):
    def __init__(self):
        super().__init__('random_joint_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.timer = self.create_timer(5.0, self.publish_random_joint_values)

    def publish_random_joint_values(self):
        msg = Float64MultiArray()
        msg.data = [random.uniform(-1.0, 1.0) for _ in range(5)]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = userCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()