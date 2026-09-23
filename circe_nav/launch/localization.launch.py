#!/usr/bin/env python3
"""State estimation: robot_localization EKF fusing wheel odometry.

    ros2 launch circe_nav localization.launch.py

Ported from roverrobotics_driver/launch/robot_localizer.launch.py.

Publishes: /odometry/filtered, and the odom -> base_link transform.
Subscribes: /odometry/wheels (from circe_bringup's rover.launch.py)

*** THIS NODE OWNS odom -> base_link. ***
Nothing else may publish that edge. In particular
circe_bringup/config/rover_zero_4wd.yaml keeps the motor driver's publish_tf false,
and there is no static publisher for it outside bench_mode. Two publishers on this
edge is the single most common way to get a robot that jitters or sits at the origin.

See config/localization_ekf.yaml for the open question about the imu0 input.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = Path(
        get_package_share_directory('circe_nav'), 'config', 'localization_ekf.yaml')
    assert default_params.is_file(), f'missing {default_params}'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock instead of wall time. True only under Gazebo.'),
        DeclareLaunchArgument(
            'params_file', default_value=str(default_params),
            description='robot_localization EKF parameters'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            # Must match the top-level key in localization_ekf.yaml.
            name='ekf_filter_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
