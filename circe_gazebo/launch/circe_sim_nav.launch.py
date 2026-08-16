#!/usr/bin/env python3
"""CIRCE in Gazebo, with the full navigation stack on top.

Layered exactly like circe_bringup.launch.py, so the same debugging discipline
applies: bring it up bottom-up and stop at the layer you are debugging.

    ros2 launch circe_gazebo circe_sim_nav.launch.py                       # sim + EKF + SLAM + Nav2
    ros2 launch circe_gazebo circe_sim_nav.launch.py use_nav2:=false       # sim + EKF + SLAM
    ros2 launch circe_gazebo circe_sim_nav.launch.py use_slam:=false use_nav2:=false   # sim + EKF
    ros2 launch circe_gazebo circe_sim.launch.py                           # sim alone

Do not skip ahead to Nav2. Almost every "Nav2 doesn't work" report is really a bad TF
tree or a bad /scan, and both are far easier to see in isolation. That is as true in
sim as on the robot -- more so, because in sim it is tempting to assume the physics is
the problem when it is really the frame graph.

*** WHY THIS REUSES circe_nav UNCHANGED ***
localization.launch.py, slam.launch.py and navigation.launch.py each already declare a
use_sim_time argument and apply it as an override dict AFTER their params file, so the
launch value beats the `use_sim_time: false` sitting in the YAML. Nav2's params go
through nav2_bringup's RewrittenYaml, which rewrites all of its use_sim_time keys the
same way. So there is no sim-specific copy of localization_ekf.yaml or nav2_params.yaml
anywhere, and there should not be one -- a forked params file is how sim and hardware
quietly drift apart.

circe_bringup.launch.py is NOT reused: it declares no use_sim_time and forwards nothing
to its includes, and its first layer starts the real motor driver.

Frame ownership, identical to hardware:
    map -> odom             slam_toolbox           (use_slam:=true)
    odom -> base_link       robot_localization EKF
    base_link -> ...        robot_state_publisher, from the URDF
The gz DiffDrive plugin is deliberately not on that list; see config/gz_bridge.yaml.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, condition=None, arguments=None):
    """Include a launch file with its launch configurations isolated.

    *** THE GroupAction IS LOAD-BEARING. DO NOT REMOVE IT. ***

    IncludeLaunchDescription does not scope launch configurations. All three circe_nav
    launch files declare an argument named `params_file`, each defaulting to its own
    YAML, and DeclareLaunchArgument only applies its default when the configuration is
    not already set. So without scoping, the first include to run wins and every later
    one silently inherits its value:

        localization.launch.py  sets params_file = localization_ekf.yaml
        slam.launch.py          inherits it, never sees mapper_params_circe.yaml
        navigation.launch.py    inherits it, never sees nav2_params.yaml

    Nav2 then finds no controller_server key in localization_ekf.yaml, falls back to
    its compiled defaults, loads dwb_core::DWBLocalPlanner instead of MPPI, finds no
    critics for it, and aborts the whole bringup with "No critics defined for
    FollowPath" - an error that says nothing about params files at all.

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

    # Every node below Gazebo has to agree that time comes from /clock. Anything left
    # on wall clock will throw TF extrapolation errors as soon as the sim's real time
    # factor drifts from 1.0, which it will.
    sim_time = {'use_sim_time': 'true'}

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='circe_indoor.sdf',
            description='World file in circe_gazebo/worlds'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo GUI'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz with the simulation layout'),
        DeclareLaunchArgument(
            'use_slam', default_value='true',
            description='Start slam_toolbox. Needs a live odom -> base_link first, '
                        'which the EKF below provides.'),
        DeclareLaunchArgument(
            'use_nav2', default_value='true',
            description='Start Nav2. Only meaningful once TF, /scan and odometry are '
                        'all verified clean.'),

        # ---- Layer 1: simulated hardware ---------------------------------------------
        _include('circe_gazebo', 'circe_sim.launch.py', arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()),

        # ---- Layer 2: state estimation -----------------------------------------------
        # Owns odom -> base_link. Fuses /odometry/wheels from the DiffDrive plugin and,
        # unlike on the real robot, an actual /imu/data from the simulated IMU.
        _include('circe_nav', 'localization.launch.py', arguments=sim_time.items()),

        # ---- Layer 3: SLAM -----------------------------------------------------------
        _include('circe_nav', 'slam.launch.py',
                 condition=IfCondition(use_slam), arguments=sim_time.items()),

        # ---- Layer 4: Nav2 -----------------------------------------------------------
        _include('circe_nav', 'navigation.launch.py',
                 condition=IfCondition(use_nav2), arguments=sim_time.items()),
    ])
