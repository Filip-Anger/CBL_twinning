from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

   gazebo = IncludeLaunchDescription(
       PythonLaunchDescriptionSource(
           PathJoinSubstitution([
               FindPackageShare('turtlebot3_gazebo'),
               'launch',
               'turtlebot3_world.launch.py'
           ])
       )
   )

   spawn_robot = Node(
       package='ros_gz_sim',
       executable='create',
       arguments=[
           '-topic', '/sim/robot_description',
           '-name', 'turtlebot3',
           '-x', '0',
           '-y', '0',
           '-z', '0.1'
       ],
       output='screen'
   )

   return LaunchDescription([
       PushRosNamespace('sim'),
       gazebo,
       spawn_robot
   ])
