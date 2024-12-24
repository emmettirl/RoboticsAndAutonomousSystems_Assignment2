#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur3_move',
            executable='userCommandNode.py',
            name='user_command_node',
            output='screen',
            parameters=[]
        ),
    ])