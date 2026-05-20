# Copyright 2024 falinux
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
"""Launch SLAM Toolbox (online async) + Border Recorder node."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for border recording with SLAM."""
    # ---- SLAM Toolbox (online async) ----
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            ])
        ),
    )

    # ---- Border Recorder Node ----
    border_recorder_node = Node(
        package='border_recorder',
        executable='border_recorder_node',
        name='border_recorder_node',
        output='screen',
        parameters=[
            {'odom_topic': '/odom'},
            {'min_distance': 0.1},
            {'output_dir': '~/border_maps'},
            {'simplify_tolerance': 0.05},
        ],
    )

    return LaunchDescription([
        slam_toolbox_launch,
        border_recorder_node,
    ])
