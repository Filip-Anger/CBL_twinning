from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   return LaunchDescription([
       Node(
           package='tb3_safety_stop',
           executable='twin_safety_node',
        #    executable='twin_safety_node_back',
           output='screen',
           parameters=[
               {'real_scan_topic': '/scan'},
               {'sim_scan_topic': '/sim/scan'},
               {'input_cmd_topic': '/cmd_vel_raw'},
               {'real_cmd_topic': '/cmd_vel'},
               {'sim_cmd_topic': '/sim/cmd_vel'},
               {'stop_distance': 0.45},
           ]
       )
   ])
