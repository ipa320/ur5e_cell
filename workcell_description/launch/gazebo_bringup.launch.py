#!/usr/bin/env python3
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'workcell_description'
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_urdf = os.path.join(get_package_share_directory(
        package_name), 'urdf', 'workcell.urdf.xacro')
    robot_urdf = xacro.process_file(robot_urdf)
    robot_description = robot_urdf.toxml()
    rviz_config = os.path.join(get_package_share_directory(
        package_name), 'config', 'config.rviz')

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

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory(
            'gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    spawn_entity = Node(package='gazebo_ros',
                        executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur5e_workcell'],
                        output='screen')

    nodes_to_start = [gazebo,
                      robot_state_pub_node,
                      rviz_node,
                      spawn_entity, ]

    return LaunchDescription(declared_arguments+nodes_to_start)
