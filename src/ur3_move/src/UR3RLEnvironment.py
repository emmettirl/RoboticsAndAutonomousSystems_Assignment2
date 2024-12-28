#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
import random
import time

SLEEP_TIME = 2
STEP_SIZE = 0.2
MAX_ITERATIONS = 100
LEARNING_RATE = 0.1
DISCOUNT_FACTOR = 0.99
EXPLORATION_RATE = 0.1
GOAL_POSITION = np.array([0.03, 0.385, 0.03])
DISTANCE_THRESHOLD = 0.02
REWARD_GOAL = 100
REWARD_STEP = -1


class UR3RLEnvironment(Node):
    def __init__(self):
        super().__init__('ur3_rl_environment')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_position_controller/follow_joint_trajectory')
        self.get_logger().info('UR3RLEnvironment initialized')
        self.joint_positions = [0.0] * 6
        self.goal_position = GOAL_POSITION
        self.max_iterations = MAX_ITERATIONS
        self.current_iteration = 0
        self.q_table = {}
        self.alpha = LEARNING_RATE
        self.gamma = DISCOUNT_FACTOR
        self.epsilon = EXPLORATION_RATE
        self.initial_distance_to_goal = None

    def reset(self):
        """Reset the environment to the initial state."""
        self.reset_environment()
        self.joint_positions = [0.0] * 6
        self.current_iteration = 0
        self.initial_distance_to_goal = np.linalg.norm(self.calculate_end_effector_position() - self.goal_position)
        return self.get_obs()

    def step(self, action):
        """Take an action in the environment and return the new state, reward, and done flag."""
        self.set_action(action)
        state = self.get_obs()
        reward = self.compute_reward()
        done = self.is_done()
        return state, reward, done

    def set_action(self, action):
        """Define what each action will do."""
        joint_index = action // 2
        direction = 1 if action % 2 == 0 else -1
        self.joint_positions[joint_index] += direction * STEP_SIZE
        self.send_goal(self.joint_positions)

    def get_obs(self):
        """Return current observation/state."""
        return self.joint_positions

    def is_done(self):
        """Return true if current episode has finished, false otherwise."""
        end_effector_position = self.calculate_end_effector_position()
        distance_to_goal = np.linalg.norm(end_effector_position - self.goal_position)
        done = distance_to_goal < DISTANCE_THRESHOLD
        if done:
            self.get_logger().info(
                f'Goal achieved - End effector position: {end_effector_position}, Distance to goal: {distance_to_goal}')
        return done or self.current_iteration >= self.max_iterations

    def compute_reward(self):
        """Return the reward for each step."""
        if self.is_done():
            end_effector_position = self.calculate_end_effector_position()
            distance_to_goal = np.linalg.norm(end_effector_position - self.goal_position)
            if distance_to_goal < DISTANCE_THRESHOLD:
                return REWARD_GOAL
        return REWARD_STEP

    def reset_environment(self):
        """Return robot to its initial pose (all joints at 0.0 radians)."""
        self.send_goal([0.0] * 6)

    def send_goal(self, joint_positions):
        """Send a goal to the robot."""
        self._action_client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = rclpy.duration.Duration(seconds=1.0).to_msg()
        goal_msg.trajectory.points = [point]

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pass

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Goal reached successfully')

    def calculate_end_effector_position(self):
        """Calculate the end effector position based on the current joint positions."""

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
            (self.joint_positions[0], 0.0, 0.1, np.pi / 2),
            (self.joint_positions[1], 0.0, 0.5, 0.0),
            (self.joint_positions[2], 0.0, 0.3, 0.0),
            (self.joint_positions[3], 0.0, 0.2, np.pi / 2),
            (self.joint_positions[4], 0.0, 0.1, -np.pi / 2),
            (self.joint_positions[5], 0.0, 0.1, 0.0)
        ]

        # Initialize the final transformation matrix as an identity matrix
        final_transformation = np.eye(4)

        # Multiply the transformation matrices of each joint
        for params in dh_params:
            final_transformation = np.dot(final_transformation, transformation_matrix(*params))

        # Extract the position of the end effector from the final transformation matrix
        end_effector_position = final_transformation[:3, 3]
        return end_effector_position

    def discretize_state(self, state):
        """Discretize the continuous state."""
        return tuple(np.round(state, 1))

    def choose_action(self, state):
        """Choose an action based on epsilon-greedy policy."""
        state = tuple(state)  # Convert state to a tuple
        if random.uniform(0, 1) < self.epsilon:
            action = random.randint(0, 11)  # 6 joints * 2 directions
        else:
            action = max(self.q_table.get(state, {}), key=self.q_table.get(state, {}).get,
                         default=random.randint(0, 11))
        return action

    def update_q_table_q_learning(self, state, action, reward, next_state):
        """Update Q-table using Q-learning."""
        state = self.discretize_state(state)
        next_state = self.discretize_state(next_state)
        if state not in self.q_table:
            self.q_table[state] = {a: 0 for a in range(12)}
        if next_state not in self.q_table:
            self.q_table[next_state] = {a: 0 for a in range(12)}
        best_next_action = max(self.q_table[next_state], key=self.q_table[next_state].get)
        td_target = reward + self.gamma * self.q_table[next_state][best_next_action]
        td_error = td_target - self.q_table[state][action]
        self.q_table[state][action] += self.alpha * td_error

    def update_q_table_sarsa(self, state, action, reward, next_state, next_action):
        """Update Q-table using SARSA."""
        state = self.discretize_state(state)
        next_state = self.discretize_state(next_state)
        if state not in self.q_table:
            self.q_table[state] = {a: 0 for a in range(12)}
        if next_state not in self.q_table:
            self.q_table[next_state] = {a: 0 for a in range(12)}
        td_target = reward + self.gamma * self.q_table[next_state][next_action]
        td_error = td_target - self.q_table[state][action]
        self.q_table[state][action] += self.alpha * td_error

    def train(self, num_episodes):
        for episode in range(num_episodes):
            self.get_logger().info(f'Starting episode {episode + 1}')
            state = self.reset()
            done = False
            while not done:
                action = self.choose_action(state)
                next_state, reward, done = self.step(action)
                self.update_q_table_q_learning(state, action, reward, next_state)
                state = next_state
                self.current_iteration += 1  # Increment here
                distance_to_goal = np.linalg.norm(self.calculate_end_effector_position() - self.goal_position)
                percentage_covered = 100 * (1 - distance_to_goal / self.initial_distance_to_goal)
                self.get_logger().info(
                    f'Iteration: {self.current_iteration}, Reward {reward:.2f} Distance to goal: {distance_to_goal:.2f}, Percentage covered: {percentage_covered:.2f}%')
                time.sleep(SLEEP_TIME)
            self.get_logger().info(f'Episode {episode + 1} finished')

def main(args=None):
    rclpy.init(args=args)
    rl_environment = UR3RLEnvironment()
    rl_environment.get_logger().info('Spinning node...')
    rl_environment.train(num_episodes=100)
    rclpy.spin(rl_environment)

if __name__ == '__main__':
    main()