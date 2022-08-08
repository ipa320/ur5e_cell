#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_urdf = os.path.join(get_package_share_directory('workcell_description'), 'urdf', 'workcell.urdf.xacro')
    robot_urdf = xacro.process_file(robot_urdf)
    robot_description = robot_urdf.toxml()
    rviz_config = os.path.join(get_package_share_directory('workcell_description'), 'config', 'config.rviz')
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=rviz_config,
            description="rviz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description",
            default_value=robot_description,
            description="robot_description file",
        )
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {"use_sim_time": use_sim_time}, 
            {"robot_description": robot_description},
        ],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', str(rviz_config)],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        name='joint_state_publisher_gui',
        parameters=[{"use_sim_time": use_sim_time},],
    )

    nodes_to_start=[
        robot_state_pub_node,
        rviz_node,
        joint_state_publisher_gui_node,
    ]

    return LaunchDescription(declared_arguments+nodes_to_start)

