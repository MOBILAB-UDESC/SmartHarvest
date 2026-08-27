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
)
from launch.events import matches_action
from launch_ros.actions import (
    LifecycleNode,
    Node
)
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def autostart(lifecycle_node, label):
    """Drive a lifecycle node up to the active state."""
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lifecycle_node),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lifecycle_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg=f'{label} node is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lifecycle_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    return [configure_event, activate_event]


def generate_launch_description():

    sh_bringup_pkg = get_package_share_directory('sh_bringup')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    sh_perception_server = LifecycleNode(
        package='sh_perception_server',
        executable='sh_perception_server',
        name='sh_perception_server',
        output='screen',
        namespace=namespace,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ]
    )

    sh_planning_scene_handler = Node(
        package='sh_planning_scene_handler',
        executable='sh_planning_scene_handler',
        name='sh_planning_scene_handler',
        output='screen',
        namespace=namespace,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ]
    )

    sh_move_group_server = LifecycleNode(
        package='sh_move_group_server',
        executable='sh_move_group_server',
        name='sh_move_group_server',
        output='screen',
        namespace=namespace,
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ]
    )

    detection_image_bridge_node = Node(
        condition=IfCondition(use_sim_time),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='arm_camera_bridge',
        output='screen',
        arguments=['/perception_image@sensor_msgs/msg/Image]gz.msgs.Image'],
    )

    args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [sh_bringup_pkg, 'config', 'sh_params.yaml']
            ),
            description='Full path to the config file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
    ]

    return LaunchDescription([
        *args,
        sh_perception_server,
        *autostart(sh_perception_server, 'PerceptionServer'),
        sh_planning_scene_handler,
        sh_move_group_server,
        *autostart(sh_move_group_server, 'MoveGroupServer'),
        detection_image_bridge_node
    ])
