#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class UR3MoveNode(Node):
    def __init__(self):
        super().__init__('ur3_move_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.start_time = time.time()

    def timer_callback(self):
        t = time.time() - self.start_time

        shoulder_pan_joint = (np.pi / 2) * (1 - np.cos(0.2 * np.pi * t))
        shoulder_lift_joint = (np.pi / 4) * (np.cos(0.4 * np.pi * t) - 1)
        elbow_joint = (np.pi / 2) * np.sin(0.2 * np.pi * t)

        joint_positions = [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, 0.0, 0.0, 0.0]

        msg = Float64MultiArray()
        msg.data = joint_positions
        self.publisher_.publish(msg)

        self.get_logger().info(f'Published joint positions: {joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = UR3MoveNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()