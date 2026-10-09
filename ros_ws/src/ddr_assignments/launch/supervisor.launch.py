from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    supervisor = Node(
        package='ddr_assignments',
        executable='sim_supervisor',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        supervisor
    ])