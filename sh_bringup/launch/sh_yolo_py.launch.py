from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():

    sh_bringup_pkg = get_package_share_directory('sh_bringup')

    yolo_server = Node(
        package='sh_rgbd_perception_pipeline',
        executable='yolo_server.py',
        output='screen',
        namespace='',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('params_file')
        ]
    )

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [sh_bringup_pkg, 'config', 'sh_params.yaml']
            ),
            description='Full path to the config file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
    ]

    return LaunchDescription([
        *args,
        yolo_server,
    ])
