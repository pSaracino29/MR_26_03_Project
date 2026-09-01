import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    bringup_dir = get_package_share_directory('ddr_bringup')

    # ---------- Gazebo ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ddr_description"), "launch", "gazebo.launch.py"))
    )

    # --------- SLAM Toolbox ----------
    # slam_toolbox_node = Node(
    #     package="slam_toolbox",
    #     executable='async_slam_toolbox_node', PER LA FASE DI MAPPATURA, PER LA FASE DI LOCALIZZAZIONE USARE localization_slam_toolbox_node
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[
    #         os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml'),
    #         {'use_sim_time': True}
    #     ]
    # )

    localization_toolbox_node = Node(
            package="slam_toolbox",
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                os.path.join(bringup_dir, 'config', 'mapper_params_online_async.yaml'),
                {'use_sim_time': True},
                {'map_file_name': os.path.join(bringup_dir, 'maps', 'my_map_serial')}
            ]
        )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'slam.rviz')]
    )


    return LaunchDescription([
            gazebo,
            #slam_toolbox_node,
            localization_toolbox_node,
            rviz_node
        ])