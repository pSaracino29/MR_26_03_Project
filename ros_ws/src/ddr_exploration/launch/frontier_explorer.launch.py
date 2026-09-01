#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([

        # ------------------------------------------
        # 1. FRONTIER DETECTOR
        # ------------------------------------------
        Node(
            package='ddr_exploration',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # ------------------------------------------
        # 2. FRONTIER SELECTOR
        # ------------------------------------------
        Node(
            package='ddr_exploration',
            executable='frontier_selector',
            name='frontier_selector',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        
    ])