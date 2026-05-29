# Copyright 2024 falinux
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
"""Launch Border Guard geofence node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for border guard geofence."""
    border_guard_node = Node(
        package='border_guard',
        executable='border_guard_node',
        name='border_guard_node',
        output='screen',
        parameters=[
            {'polygon_file': '~/border_maps/border_polygon.csv'},
            {'odom_topic': '/odom'},
            {'input_cmd_topic': '/cmd_vel_raw'},
            {'output_cmd_topic': '/cmd_vel'},
        ],
    )

    return LaunchDescription([
        border_guard_node,
    ])
