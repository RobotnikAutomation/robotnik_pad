from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    config_file = "/home/robot/robot_ws/install/share/robotnik_pad/config/ps5.yaml"

    return LaunchDescription([
        Node(
            package='robotnik_pad',
            executable='robotnik_pad',
            output='screen',
            name='robotnik_pad',
            parameters=[config_file]
        )
    ])
