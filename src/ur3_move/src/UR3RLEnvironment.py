#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
import random
import time
from sensor_msgs.msg import JointState
import threading

# Constants
SLEEP_TIME = 3
STEP_SIZE = 0.75
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

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,))
        self.spin_thread.start()

    def joint_state_callback(self, msg):
        joint_positions_dict = dict(zip(msg.name, msg.position))
        self.joint_positions = [
            joint_positions_dict.get('shoulder_pan_joint', 0.0),
            joint_positions_dict.get('shoulder_lift_joint', 0.0),
            joint_positions_dict.get('elbow_joint', 0.0),
            joint_positions_dict.get('wrist_1_joint', 0.0),
            joint_positions_dict.get('wrist_2_joint', 0.0),
            joint_positions_dict.get('wrist_3_joint', 0.0),
        ]

    def reset(self):
        # Reset the environment to the initial state.
        self.send_goal([0.0] * 6)
        time.sleep(SLEEP_TIME * 2)
        self.joint_positions = [0.0] * 6
        self.current_iteration = 0
        self.initial_distance_to_goal = np.linalg.norm(self.calculate_end_effector_position() - self.goal_position)
        return self.get_obs()

    def step(self, action):
        self.set_action(action)
        state = self.get_obs()
        reward = self.compute_reward()
        done = self.is_done()
        return state, reward, done

    def set_action(self, action):
        joint_index = action // 2
        direction = 1 if action % 2 == 0 else -1
        self.joint_positions[joint_index] += direction * STEP_SIZE
        self.joint_positions[joint_index] = np.clip(self.joint_positions[joint_index], -np.pi, np.pi)
        self.send_goal(self.joint_positions)

    def get_obs(self):
        return self.joint_positions

    def is_done(self):
        end_effector_position = self.calculate_end_effector_position()
        distance_to_goal = np.linalg.norm(end_effector_position - self.goal_position)
        done = distance_to_goal < DISTANCE_THRESHOLD
        if done:
            self.get_logger().info(
                f'Goal achieved - End effector position: {end_effector_position}, Distance to goal: {distance_to_goal}')
        return done or self.current_iteration >= self.max_iterations

    def compute_reward(self):
        if self.is_done():
            end_effector_position = self.calculate_end_effector_position()
            distance_to_goal = np.linalg.norm(end_effector_position - self.goal_position)
            if distance_to_goal < DISTANCE_THRESHOLD:
                return REWARD_GOAL
        return REWARD_STEP


    def send_goal(self, joint_positions):
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
            # self.get_logger().info('Goal reached successfully')
            pass



    def calculate_end_effector_position(self):
        def dh_transform(theta, d, a, alpha):
            return np.array([
                [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
                [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
                [0, np.sin(alpha), np.cos(alpha), d],
                [0, 0, 0, 1]
            ])

        d1 = 0.1519
        a2 = -0.24365
        a3 = -0.21325
        d4 = 0.11235
        d5 = 0.08535
        d6 = 0.0819

        theta1, theta2, theta3, theta4, theta5, theta6 = self.joint_positions

        # Compute transformation matrices
        T1 = dh_transform(theta1, d1, 0, np.pi / 2)
        T2 = dh_transform(theta2, 0, a2, 0)
        T3 = dh_transform(theta3, 0, a3, 0)
        T4 = dh_transform(theta4, d4, 0, np.pi / 2)
        T5 = dh_transform(theta5, d5, 0, -np.pi / 2)
        T6 = dh_transform(theta6, d6, 0, 0)

        # Compound transformation
        T_final = T1 @ T2 @ T3 @ T4 @ T5 @ T6

        # Extract position from the final transformation matrix
        end_effector_position = T_final[:3, 3]
        # print (end_effector_position)
        self.get_logger().info(
            f'End effector position: {end_effector_position}, Joint positions: {self.joint_positions}, GOAL_POSITION: {self.goal_position}')
        return end_effector_position

    def discretize_state(self, state):
        return tuple(np.round(state, 1))

    def choose_action(self, state):
        state = tuple(state)  # Convert state to a tuple
        if random.uniform(0, 1) < self.epsilon:
            action = random.randint(0, 11)  # 6 joints * 2 directions
        else:
            action = max(self.q_table.get(state, {}), key=self.q_table.get(state, {}).get,
                         default=random.randint(0, 11))
        return action

    def update_q_table_q_learning(self, state, action, reward, next_state):
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
            self.reset()
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