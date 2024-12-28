#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import threading
import time
import pandas as pd
import numpy as np


result_wait_time = 10


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
        self.desired_joint_positions = {}
        self.get_logger().info('Node has been started. Waiting for user input...')
        self.spin_thread = threading.Thread(target=self.spin)
        self.spin_thread.start()

    def joint_state_callback(self, msg):
        for name, position in zip(msg.name, msg.position):
            self.current_joint_positions[name] = position

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
                        desired_position = current_position + relative_move
                        self.desired_joint_positions[joint] = desired_position
                        joint_values.append(desired_position)
                    except ValueError:
                        self.get_logger().error('Invalid input. Please enter a number.')
                        break
                else:
                    msg = Float64MultiArray()
                    msg.data = joint_values
                    self.publisher_.publish(msg)
                    formatted_values = [f"{val:.2f}" for val in msg.data]
                    self.get_logger().info(f'Published: {formatted_values}')
                    time.sleep(result_wait_time)
                    self.print_final_positions()
        except KeyboardInterrupt:
            self.get_logger().info('Node interrupted and shutting down...')

    def print_final_positions(self):
        data = {
            'Joint': [],
            'Actual Position': [],
            'Desired Position': [],
            'Error': []
        }
        for joint in self.joint_names:
            actual_position = self.current_joint_positions.get(joint, 0.0)
            desired_position = self.desired_joint_positions.get(joint, 0.0)
            error = actual_position - desired_position
            data['Joint'].append(joint)
            data['Actual Position'].append(actual_position)
            data['Desired Position'].append(desired_position)
            data['Error'].append(error)

        df = pd.DataFrame(data)
        df = df.round({'Actual Position': 4, 'Desired Position': 4, 'Error': 4})
        print('\n', 'Result:', '\n', df, '\n')

        end_effector_transformation = self.get_end_effector_pose()
        df_transformation = pd.DataFrame(end_effector_transformation, columns=['x', 'y', 'z', 'w'],
                                         index=['x', 'y', 'z', 'w'])
        df_transformation = df_transformation.round(4)
        print('End effector transformation matrix:')
        print(df_transformation)


    def get_end_effector_pose(self):
        # Define the transformation matrices for each joint
        def transformation_matrix(joint_angle, joint_offset, link_length, twist_angle):
            return np.array([
                [np.cos(joint_angle), -np.sin(joint_angle) * np.cos(twist_angle),
                 np.sin(joint_angle) * np.sin(twist_angle), link_length * np.cos(joint_angle)],
                [np.sin(joint_angle), np.cos(joint_angle) * np.cos(twist_angle),
                 -np.cos(joint_angle) * np.sin(twist_angle), link_length * np.sin(joint_angle)],
                [0, np.sin(twist_angle), np.cos(twist_angle), joint_offset],
                [0, 0, 0, 1]
            ])

        # Example DH parameters for a 6-DOF robot arm (you need to replace these with your actual parameters)
        dh_params = [
            (self.current_joint_positions.get('shoulder_pan_joint', 0.0), 0.0, 0.1, np.pi / 2),
            (self.current_joint_positions.get('shoulder_lift_joint', 0.0), 0.0, 0.5, 0.0),
            (self.current_joint_positions.get('elbow_joint', 0.0), 0.0, 0.3, 0.0),
            (self.current_joint_positions.get('wrist_1_joint', 0.0), 0.0, 0.2, np.pi / 2),
            (self.current_joint_positions.get('wrist_2_joint', 0.0), 0.0, 0.1, -np.pi / 2),
            (self.current_joint_positions.get('wrist_3_joint', 0.0), 0.0, 0.1, 0.0)
        ]

        # Initialize the final transformation matrix as an identity matrix
        final_transformation = np.eye(4)

        # Multiply the transformation matrices of each joint
        for params in dh_params:
            final_transformation = np.dot(final_transformation, transformation_matrix(*params))

        return final_transformation


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