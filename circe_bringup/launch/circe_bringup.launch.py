#!/usr/bin/env python3
"""CIRCE top-level bring-up. One entry point, layered by launch argument.

The layers stack bottom-up, exactly as BRINGUP_CHECKLIST.md prescribes, and each is
independently switchable so you can stop at whichever one you are debugging:

    base + sensors + EKF                    ros2 launch circe_bringup circe_bringup.launch.py
    ... + SLAM                              ... use_slam:=true
    ... + Nav2                              ... use_slam:=true use_nav2:=true
    ... + gamepad teleop                    ... use_teleop:=true
    no rover attached, lidar on the bench   ... bench_mode:=true

Do not skip ahead to Nav2. Almost every "Nav2 doesn't work" report is really a bad TF
tree or a bad /scan, and both are far easier to see in isolation.

Frame ownership -- exactly one publisher per edge, which is the whole point:
    map -> odom             slam_toolbox           (use_slam:=true)
    odom -> base_link       robot_localization EKF (rover_zero_4wd.yaml keeps the
                                                    driver's publish_tf false)
    base_link -> ...        robot_state_publisher, from the URDF
    camera_link -> camera_* realsense2_camera, from factory calibration
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, condition=None, arguments=None):
    """Include a launch file with its launch configurations isolated.

    *** THE GroupAction IS LOAD-BEARING. DO NOT REMOVE IT. ***

    IncludeLaunchDescription does not scope launch configurations, and
    DeclareLaunchArgument only applies its default when the configuration is not
    already set. All three circe_nav launch files declare an argument named
    `params_file`, each defaulting to a different YAML, so without scoping the first
    include to run wins and the rest silently inherit its value:

        localization.launch.py  sets params_file = localization_ekf.yaml
        slam.launch.py          inherits it, never sees mapper_params_circe.yaml
        navigation.launch.py    inherits it, never sees nav2_params.yaml

    Nav2 then finds no controller_server key in localization_ekf.yaml, falls back to
    its compiled defaults, loads dwb_core::DWBLocalPlanner instead of the configured
    MPPI, finds no critics for it, and aborts the entire bringup with "No critics
    defined for FollowPath" - an error that never mentions params files.

    GroupAction defaults to scoped=True, which pushes and pops the configuration scope
    around each include so each one gets its own defaults back.
    """
    return GroupAction(
        condition=condition,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(get_package_share_directory(package),
                             'launch', launch_file))),
                launch_arguments=arguments or {},
            ),
        ],
    )


def generate_launch_description():
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_teleop = LaunchConfiguration('use_teleop')
    bench_mode = LaunchConfiguration('bench_mode')

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument(
            'use_slam', default_value='false',
            description='Start slam_toolbox. Needs a live odom -> base_link first, or '
                        'it blocks waiting for the transform and publishes no map.'),
        DeclareLaunchArgument(
            'use_nav2', default_value='false',
            description='Start Nav2. Only meaningful once TF, /scan and odometry are '
                        'all verified clean.'),
        DeclareLaunchArgument(
            'use_teleop', default_value='false',
            description='Start gamepad teleop from circe_control'),
        DeclareLaunchArgument(
            'bench_mode', default_value='false',
            description='Desk testing with no rover attached: skip the motor driver '
                        'and the EKF, and publish a static odom -> base_link so tools '
                        'that need a complete tree still work. NEVER set this on the '
                        'real robot -- it pins the robot at the origin and fights the '
                        'EKF, which is exactly the bug the old circe_tf package caused.'),

        # ---- Layer 1: base + sensors -------------------------------------------------
        # In bench_mode the motor driver is skipped (there is nothing to talk to) but
        # robot_state_publisher is still wanted, so the base launch is replaced by a
        # bare robot_state_publisher below.
        _include('circe_bringup', 'rover.launch.py', condition=UnlessCondition(bench_mode)),

        _include('circe_bringup', 'sensors.launch.py', arguments={
            'use_lidar': LaunchConfiguration('use_lidar'),
            'use_camera': LaunchConfiguration('use_camera'),
            'use_imu': LaunchConfiguration('use_imu'),
        }.items()),

        # ---- Layer 2: state estimation ----------------------------------------------
        _include('circe_nav', 'localization.launch.py', condition=UnlessCondition(bench_mode)),

        # ---- Layer 3: SLAM ----------------------------------------------------------
        _include('circe_nav', 'slam.launch.py', condition=IfCondition(use_slam)),

        # ---- Layer 4: Nav2 ----------------------------------------------------------
        _include('circe_nav', 'navigation.launch.py', condition=IfCondition(use_nav2)),

        # ---- Optional: teleop -------------------------------------------------------
        _include('circe_control', 'teleop.launch.py', condition=IfCondition(use_teleop)),

        # ---- bench_mode substitutes ---------------------------------------------------
        GroupAction(
            condition=IfCondition(bench_mode),
            actions=[
                _include('circe_description', 'display.launch.py', arguments={
                    # display.launch.py also starts RViz, which is what you want on the
                    # bench. It brings its own joint_state_publisher.
                }.items()),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='bench_odom_to_base_link',
                    arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link'],
                ),
            ],
        ),
    ])
