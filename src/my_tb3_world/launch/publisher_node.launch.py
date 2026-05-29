from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_tb3_world',
            executable='publisher_node',
            name='publisher_node',
            output='screen',
        )
    ])
