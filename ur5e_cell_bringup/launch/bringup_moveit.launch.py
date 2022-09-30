from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_files = []
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur5e_cell_msa_config"), "/launch", "/move_group.launch.py"]),
    )
    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare(
            "ur5e_cell_msa_config"), "/launch", "/moveit_rviz.launch.py"]),
    )

    launch_files.append(moveit_launch)
    launch_files.append(moveit_rviz_launch)

    return LaunchDescription(launch_files)
