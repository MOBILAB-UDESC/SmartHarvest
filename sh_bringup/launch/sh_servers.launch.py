from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution
)
from launch.events import matches_action
from launch_ros.actions import (
    LifecycleNode,
    Node
)
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():

    sh_bringup_pkg = get_package_share_directory('sh_bringup')

    sh_perception_server = LifecycleNode(
        package='sh_perception_server',
        executable='sh_perception_server',
        name='sh_perception_server',
        output='screen',
        namespace='',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('params_file')
        ]
    )

    configure_event_handler = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(sh_perception_server),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=sh_perception_server,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='PerceptionServer node is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(sh_perception_server),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    detection_image_bridge_node = Node(
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='arm_camera_bridge',
        output='screen',
        arguments=['/detection_image@sensor_msgs/msg/Image]gz.msgs.Image'],
    )

    sh_planning_scene_handler = Node(
        package='sh_planning_scene_handler',
        executable='sh_planning_scene_handler',
        output='screen',
        namespace='',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('params_file')
        ]
    )

    sh_move_group_server = Node(
        package='sh_move_group_server',
        executable='sh_move_group_server',
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
        sh_perception_server,
        configure_event_handler,
        activate_event_handler,
        sh_planning_scene_handler,
        # sh_move_group_server,
        # detection_image_bridge_node
    ])
