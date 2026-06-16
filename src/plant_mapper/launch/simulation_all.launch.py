import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    map_yaml_path = '/home/falinux/CBL_twinning/src/mapFiles/playground.yaml'
    map_pgm_path = '/home/falinux/CBL_twinning/src/mapFiles/playground.pgm'
    
    # 1. Include Gazebo World
    gazebo_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('my_tb3_world'),
                'launch',
                'new_world.launch.py'
            )
        ])
    )
    
    # 2. Include Navigation2
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'launch',
                'navigation2.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'True',
            'map': map_yaml_path
        }.items()
    )
    
    # 3. Include Digital Twin Package Nodes
    twin_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('plant_mapper'),
                'launch',
                'twin_system.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'map_yaml': map_yaml_path,
            'map_pgm': map_pgm_path
        }.items()
    )
    
    return LaunchDescription([
        gazebo_world_launch,
        navigation_launch,
        twin_system_launch
    ])
