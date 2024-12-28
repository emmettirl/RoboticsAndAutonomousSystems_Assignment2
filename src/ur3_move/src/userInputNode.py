#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UserInputNode(Node):
    def __init__(self):
        super().__init__('user_input_node')  # Name the node
        self.publisher_ = self.create_publisher(String, 'user_input_topic', 10)
        self.get_logger().info('Node has been started. Waiting for user input...')

    def take_user_input(self):
        try:
            while rclpy.ok():  # Ensure ROS is running
                user_input = input("Enter your message: ")
                if user_input.lower() == 'exit':  # Graceful exit
                    self.get_logger().info('Exiting...')
                    break
                # Publish the input as a String message
                msg = String()
                msg.data = user_input
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: "{user_input}"')
        except KeyboardInterrupt:
            self.get_logger().info('Node interrupted and shutting down...')

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = UserInputNode()

    try:
        node.take_user_input()  # Call the input loop
    finally:
        node.destroy_node()  # Destroy the node explicitly
        rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()
