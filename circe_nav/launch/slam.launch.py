#!/usr/bin/env python3
"""Lidar-only SLAM for CIRCE using slam_toolbox in async mode.

    ros2 launch circe_nav slam.launch.py
    ros2 launch circe_nav slam.launch.py use_rviz:=true

Ported from the my_slam package, which is not carried into this workspace: it was a
one-launch-file wrapper around slam_toolbox and belongs here with the rest of the
navigation chain.

Publishes map -> odom by scan matching /scan from the RPLidar A1.

This launch file does NOT provide odom -> base_link. That edge comes from
localization.launch.py in this package, or from circe_bringup's bench_mode static
publisher when there is no rover attached. Without one of them slam_toolbox blocks
waiting for the transform and no map ever appears -- if /map is silent, check
`ros2 run tf2_ros tf2_echo odom base_link` before touching any mapper parameter.

*** WHY THIS USES LifecycleNode AND EMITS TRANSITIONS ***

On Jazzy, async_slam_toolbox_node is a LIFECYCLE node. Starting it with a plain
launch_ros Node -- the Humble pattern, and what this file used to do -- leaves it
parked in the `unconfigured` state forever. Nothing sends it a transition, so it
never declares its parameters, never subscribes to scan_topic, and never publishes
map -> odom.

The failure is unusually quiet. The process starts, logs

    [slam_toolbox]: Node using stack size 40000000

and then says nothing ever again. It looks alive: `ros2 node list` shows it and it
answers service calls. What gives it away is that `ros2 service list` shows ONLY
lifecycle and parameter services (change_state, get_state, get_available_transitions,
...) and none of slam_toolbox's own, that `ros2 param get /slam_toolbox scan_topic`
returns "Parameter not set" even though the YAML sets it, and that /scan has no
subscriber. Diagnose it with:

    ros2 lifecycle get /slam_toolbox      # unconfigured [1] means this is the bug

Upstream's own slam_toolbox/launch/online_async_launch.py does what this file now
does, and roverrobotics_driver/launch/slam_launch.py does NOT -- it is still on the
plain-Node pattern, so do not use it as a reference here.

`use_lifecycle_manager:=true` hands control to an external nav2 lifecycle manager
instead; the autostart events below are then suppressed, matching upstream.

Tuning lives in config/mapper_params_circe.yaml.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = Path(get_package_share_directory('circe_nav'))
    default_params = pkg / 'config' / 'mapper_params_circe.yaml'
    default_rviz = pkg / 'rviz' / 'slam.rviz'
    assert default_params.is_file(), f'missing {default_params}'

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')

    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        # Must match the top-level key in the params file.
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_lifecycle_manager': use_lifecycle_manager,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
    )

    # unconfigured -> inactive
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    # inactive -> active, once configuring has actually finished
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg='[LifecycleLaunch] slam_toolbox is activating.'),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of wall time. True only under Gazebo.'),
        DeclareLaunchArgument(
            'params_file', default_value=str(default_params),
            description='slam_toolbox mapper parameters'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='Open RViz with the SLAM layout'),
        DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Configure and activate slam_toolbox automatically. Without '
                        'this it sits in the unconfigured state and publishes '
                        'nothing. Ignored when use_lifecycle_manager is true.'),
        DeclareLaunchArgument(
            'use_lifecycle_manager', default_value='false',
            description='Let an external nav2 lifecycle manager drive the node '
                        'instead of the autostart events here.'),

        slam,
        configure_event,
        activate_event,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_slam',
            output='screen',
            arguments=['-d', str(default_rviz)],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
