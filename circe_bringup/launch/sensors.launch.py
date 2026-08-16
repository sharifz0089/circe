#!/usr/bin/env python3
"""CIRCE sensor suite: RPLidar A1 + RealSense D456 (+ optional BNO055).

Replaces roverrobotics_driver/launch/accessories.launch.py. That file opened its own
YAML at launch time and branched on an `active:` key inside it, which meant "run the
lidar without the camera" required editing a config file and rebuilding. Sensors are
now plain launch arguments, so the bottom-up bring-up in the checklist is one command
each:

    ros2 launch circe_bringup sensors.launch.py use_camera:=false   # lidar alone
    ros2 launch circe_bringup sensors.launch.py use_lidar:=false    # camera alone
    ros2 launch circe_bringup sensors.launch.py                     # both

Topics and frames
-----------------
Lidar:  /scan                       in frame lidar_link
Camera: /camera/camera/depth/image_rect_raw
        /camera/camera/color/image_raw
        /camera/camera/imu
        frames camera_link -> camera_{depth,color,accel,gyro}_* (published by the driver)

The camera runs as namespace `camera` + node name `camera`, i.e. fully-qualified
/camera/camera, which is the upstream realsense2_camera convention and gives the
/camera/camera/... topic prefix its own documentation uses. Note that the old
accessories.launch.py used name="realsense" with no namespace, so topics there were
actually /realsense/... -- BRINGUP_CHECKLIST.md Step 2 documented /camera/camera/...
and was wrong for that launch path. It is correct for this one.

Frames are unaffected by any of this: camera_link is composed by the driver from
`camera_name` in config/realsense_d456.yaml, not from the node name.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('circe_bringup'), 'config')

    lidar_config = config / 'lidar_a1.yaml'
    camera_config = config / 'realsense_d456.yaml'
    imu_config = config / 'bno055.yaml'
    for f in (lidar_config, camera_config, imu_config):
        assert f.is_file(), f'missing sensor config: {f}'

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_lidar', default_value='true',
            description='Start the RPLidar A1 (needs /dev/rplidar)'),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            description='Start the RealSense D456 (needs a USB 3 port)'),
        DeclareLaunchArgument(
            'use_imu', default_value='false',
            description='Start the external BNO055 IMU. Off by default: no BNO055 is '
                        'fitted and the `bno055` package is not installed.'),

        # sllidar_node's own defaults are wrong for an A1 (1000000 baud,
        # frame_id "laser_frame"), so every value comes from lidar_a1.yaml. That file
        # is keyed on /** precisely so renaming this node cannot silently drop them.
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='rplidar_a1',
            parameters=[lidar_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_lidar')),
        ),

        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',
            name='camera',
            parameters=[camera_config],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),

        # Remap to /imu/data because that is what the EKF's imu0 input expects
        # (circe_nav/config/localization_ekf.yaml).
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            parameters=[imu_config],
            remappings=[('/imu', '/imu/data')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_imu')),
        ),
    ])
