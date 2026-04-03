from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    sh_bringup_pkg = get_package_share_directory('sh_bringup')

    sh_detection_server_cpp_node = LifecycleNode(
        package='sh_bt_core',
        executable='sh_bt_core',
        name='sh_bt_core',
        output='screen',
        namespace='',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('params_file')
        ],
        # arguments=[
        #     '--ros-args',
        #     '--log-level', 'debug',
        #     '--log-level', 'rcl:=INFO'
        # ]
    )

    configure_event_handler = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(sh_detection_server_cpp_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sh_detection_server_cpp_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(sh_detection_server_cpp_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    args = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [sh_bringup_pkg, 'config', 'sh_params.yaml']
            ),
            description='Full path to the behavior tree config file'
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
        sh_detection_server_cpp_node,
        configure_event_handler,
        activate_event_handler
    ])
