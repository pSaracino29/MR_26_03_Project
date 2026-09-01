#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ddr_bringup'),
                'launch',
                'navigation.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    frontier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ddr_exploration'),
                'launch',
                'frontier_explorer.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()

    )

    return LaunchDescription([
        simulation,
        frontier,
    ])