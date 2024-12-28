#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
import time

class UR3MoveActionClient(Node):
    def __init__(self):
        super().__init__('ur3_move_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_position_controller/follow_joint_trajectory')
        self.start_time = time.time()
        self.get_logger().info('UR3MoveActionClient initialized')

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available, sending goal...')

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        points = []
        for t in np.linspace(0, 10, 100):
            shoulder_pan_joint = (np.pi / 2) * (1 - np.cos(0.2 * np.pi * t))
            shoulder_lift_joint = (np.pi / 4) * (np.cos(0.4 * np.pi * t) - 1)
            elbow_joint = (np.pi / 2) * np.sin(0.2 * np.pi * t)
            joint_positions = [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, 0.0, 0.0, 0.0]

            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
            points.append(point)

        goal_msg.trajectory.points = points

        self.get_logger().info(f'Sending goal: {goal_msg}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = UR3MoveActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()