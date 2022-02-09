#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('path_following'),
            'config',
            'path_following_turtlebot.yaml'))
    
    rviz_dir = LaunchConfiguration(
        'rviz_dir',
        default=os.path.join(
            get_package_share_directory('path_following'),
            'rviz',
            'pathfollowing.rviz'))
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of path file'
        ),

        Node(
            package='path_following',
            node_executable='pid_planner',
            node_name='pid_planner',
            parameters=[param_dir],
            output='screen',
            remappings=[('/ctrl_cmd', '/turtle1/cmd_vel')]),
        
        # Node(
        #     package='path_following',
        #     node_executable='pose_to_odom',
        #     node_name='pose_to_odom',
        #     parameters=[param_dir],
        #     output='screen'),
        
        Node(
            package='path_following',
            node_executable='pose_stamped_subscriber',
            node_name='pose_stamped_subscriber',
            parameters=[param_dir],
            output='screen'),
        
        Node(
            package='path_following',
            node_executable='pose_test',
            node_name='pose_test',
            output='screen'),

        Node(
            package='turtlesim',
            node_executable='turtlesim_node',
            node_name='turtlesim_node',
            output='screen'),

        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen'),
    ])