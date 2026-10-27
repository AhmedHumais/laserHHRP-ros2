"""Operator UI: RViz 2 with the laser roller panel (config/hhrp.rviz).

    ros2 launch laser_hhrp ui.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution([FindPackageShare("laser_hhrp"), "config", "hhrp.rviz"])
    return LaunchDescription([
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_config], output="screen"),
    ])
