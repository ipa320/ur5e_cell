#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'workcell_description'

    bringup = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory(package_name), 'launch', 'workcell_bringup.launch.py')]),)

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory(
            'gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        )

    spawn_entity = Node(package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'entity' ],
        output='screen')

    return LaunchDescription([
        bringup, ])

    
"""     camera_node = Node(package='v4l2_camera',
                        executable='v4l2_camera_node',
                        output='screen',
                        parameters=[{
                            'image_size': [1920,1080],
                            'camera_frame_id': 'camera_link_optical'
                            }]) """
