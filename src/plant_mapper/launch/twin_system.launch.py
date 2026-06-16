import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value='/home/falinux/CBL_twinning/src/mapFiles/playground.yaml',
        description='Path to map yaml file'
    )
    
    map_pgm_arg = DeclareLaunchArgument(
        'map_pgm',
        default_value='/home/falinux/CBL_twinning/src/mapFiles/playground.pgm',
        description='Path to map pgm file'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')
    map_pgm = LaunchConfiguration('map_pgm')
    
    # Nodes to launch
    farm_twin_node = Node(
        package='plant_mapper',
        executable='farm_twin',
        name='farm_twin',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_yaml': map_yaml,
            'map_pgm': map_pgm
        }],
        output='screen'
    )
    
    farm_navigator_node = Node(
        package='plant_mapper',
        executable='farm_navigator',
        name='farm_navigator',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    battery_node = Node(
        package='plant_mapper',
        executable='battery',
        name='battery',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_arg,
        map_pgm_arg,
        farm_twin_node,
        farm_navigator_node,
        battery_node
    ])
