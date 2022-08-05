#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    robot_urdf = os.path.join(get_package_share_directory('workcell_description'), 'urdf', 'workcell.urdf.xacro')
    robot_urdf = xacro.process_file(robot_urdf)
    robot_description = {'robot_description': robot_urdf.toxml()}

    rviz_config_file = os.path.join(get_package_share_directory('workcell_description'), 'config', 'config.rviz')

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_urdf",
            description="URDF file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            description="rviz configuration file",
        )
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        arguments=[robot_urdf],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start=[
        robot_state_pub_node,
        rviz_node
    ]

    return launch.LaunchDescription(declared_arguments+nodes_to_start)

