import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_mapper = Node(
            package='ddr_assignments',
            executable='aruco_mapper',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

    plotter = Node(
            package='ddr_assignments',
            executable='plotter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

    simulation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ddr_exploration'),
                    'launch',
                    'exploration.launch.py'
                )
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        )

    return LaunchDescription([
        simulation,
        aruco_mapper,
        plotter
    ])