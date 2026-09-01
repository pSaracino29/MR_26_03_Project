import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ddr_description = get_package_share_directory("ddr_description")
    
    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                ddr_description, "urdf", "ddr.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")
    
    world_path = PathJoinSubstitution([
            ddr_description,
            "worlds",
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )

    model_path = str(Path(ddr_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("ddr_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=",
            is_ignition
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )


    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "ddr",
            "-x", "-5.0",  
            "-y", "4.5",  
            "-z", "0.4",  
            "-R", "0.0", 
            "-P", "0.0",
            "-Y", "-1.57", # Yaw (in radians, e.g., 1.57 for 90 degrees)
        ],
        parameters = [{'use_sim_time': True}],
    )

    gz_ros2_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",

    ],
    remappings=[
        ("/lidar", "/scan"),
    ],
    parameters = [{'use_sim_time': True}],
)

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{'use_sim_time': True}],
        arguments=["-d", os.path.join(ddr_description, "rviz", "config.rviz")],
    )

    return LaunchDescription([
        model_arg,
        world_name_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gazebo,
        gz_spawn_entity,
        gz_ros2_bridge,
        ros_gz_image_bridge,
        #rviz_node
    ])