#!/usr/bin/env python3
"""Rover Zero 4WD base: motor driver + robot_state_publisher.

Ported from roverrobotics_driver/launch/zero.launch.py, with two deliberate changes:

  * the robot model comes from circe_description, not roverrobotics_description
  * accessories.launch.py is NOT included here

Upstream, zero.launch.py pulled the sensors in as a side effect, so you could not
start the base without also starting the lidar and camera. That makes the bottom-up
bring-up in the checklist impossible. Sensors now live in sensors.launch.py and both
are composed by circe_bringup.launch.py.

    ros2 launch circe_bringup rover.launch.py

Publishes: /odometry/wheels, /joint_states, /tf (base_link and below), /robot_description
Subscribes: /cmd_vel
Does NOT publish odom -> base_link -- that edge belongs to the EKF in circe_nav.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_model_path = get_package_share_path('circe_description') / 'urdf' / 'circe_rover.urdf.xacro'

    hardware_config = Path(
        get_package_share_directory('circe_bringup'), 'config', 'rover_zero_4wd.yaml')
    # Fail at launch with a clear message rather than letting the driver come up with
    # robot_type "NONE" and device_port "NONE" and report a confusing serial error.
    assert hardware_config.is_file(), f'missing rover config: {hardware_config}'

    model_arg = DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='Absolute path to the robot xacro/urdf')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str)

    return LaunchDescription([
        model_arg,

        # respawn: the serial link to the motor controller drops if the rover is
        # power-cycled or the USB re-enumerates. Upstream respawns, and so do we --
        # without it a brownout ends the drive session.
        Node(
            package='roverrobotics_driver',
            name='roverrobotics_driver',
            executable='roverrobotics_driver',
            parameters=[hardware_config],
            output='screen',
            respawn=True,
            respawn_delay=1,
        ),

        # Publishes base_link -> chassis_link -> {wheels, imu_link, lidar_link,
        # sensor_shelf_link -> camera_bottom_screw_frame -> camera_link}.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),

        # The wheel joints are `continuous` in the URDF and the driver publishes no
        # /joint_states, so without this robot_state_publisher warns about missing
        # joint states and the wheels never render.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[hardware_config],
        ),
    ])
