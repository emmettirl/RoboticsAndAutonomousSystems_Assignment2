#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class UserInputNode(Node):
    def __init__(self):
        super().__init__('user_input_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.get_logger().info('Node has been started. Waiting for user input...')
        self.joint_names = [
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

    def take_user_input(self):
        try:
            while rclpy.ok():
                joint_values = []
                for joint in self.joint_names:
                    user_input = input(f"Enter the amount to move {joint} by: ")
                    if user_input.lower() == 'exit':
                        self.get_logger().info('Exiting...')
                        return
                    try:
                        joint_values.append(float(user_input))
                    except ValueError:
                        self.get_logger().error('Invalid input. Please enter a number.')
                        break
                else:
                    msg = Float64MultiArray()
                    msg.data = joint_values
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published: {msg.data}')
        except KeyboardInterrupt:
            self.get_logger().info('Node interrupted and shutting down...')

def main(args=None):
    rclpy.init(args=args)
    node = UserInputNode()

    try:
        node.take_user_input()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()