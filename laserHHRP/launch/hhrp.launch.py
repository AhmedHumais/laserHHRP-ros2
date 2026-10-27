"""Starts the laser roller nodes.

    ros2 launch laser_hhrp hhrp.launch.py                       # real hardware
    ros2 launch laser_hhrp hhrp.launch.py simulate:=true        # no hardware
    ros2 launch laser_hhrp hhrp.launch.py hardware_trigger:=true  # Nano D3 wired to laser In1

Parameters: config/hhrp.yaml (or params_file:=...). The operator UI: ui.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulate = LaunchConfiguration("simulate")
    params = [
        LaunchConfiguration("params_file"),
        {"sync.hardware_trigger": LaunchConfiguration("hardware_trigger")},
        {"nano.port": LaunchConfiguration("nano_port")},
    ]

    def node(executable, **kwargs):
        return Node(package="laser_hhrp", executable=executable, parameters=params,
                    output="screen", **kwargs)

    return LaunchDescription([
        DeclareLaunchArgument("simulate", default_value="false",
                              description="simulate the Nano and the laser"),
        DeclareLaunchArgument("hardware_trigger", default_value="false",
                              description="true: hardware sync (Nano D3 -> laser In1); false: software sync"),
        DeclareLaunchArgument("nano_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution(
            [FindPackageShare("laser_hhrp"), "config", "hhrp.yaml"])),

        # layer 1: hardware interface (or the simulator in their place)
        node("nano_driver_node", condition=UnlessCondition(simulate)),
        node("laser_driver_node", condition=UnlessCondition(simulate)),
        node("simulator_node", condition=IfCondition(simulate)),
        # layer 2: acquisition, odometry
        node("acquisition_node"),
        node("odometry_node"),
        # layer 3: synchronization
        node("synchronizer_node"),
        # layers 4, 5: point cloud, inspection
        node("point_cloud_node"),
        node("inspection_node"),
        # layer 6: storage
        node("recorder_node"),
        # calibration, scan control + monitoring (layer 7 logic)
        node("calibration_node"),
        node("scan_controller_node"),
    ])
