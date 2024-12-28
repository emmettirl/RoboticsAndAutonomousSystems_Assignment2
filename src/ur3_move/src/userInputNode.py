#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import threading

class UserInputNode(Node):
    def __init__(self):
        super().__init__('user_input_node')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.current_joint_positions = {}
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.get_logger().info('Node has been started. Waiting for user input...')
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.start()

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position
        # self.get_logger().info(f'Updated joint positions: {self.current_joint_positions}')

    def spin(self):
        rclpy.spin(self)

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
                        relative_move = float(user_input)
                        current_position = self.current_joint_positions.get(joint, 0.0)
                        joint_values.append(current_position + relative_move)
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