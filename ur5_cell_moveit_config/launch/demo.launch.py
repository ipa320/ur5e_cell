from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur5_msa", package_name="ur5_cell_moveit_config").to_moveit_configs()
    #print(moveit_config.to_dict())
    ld = generate_demo_launch(moveit_config)
    ur_control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        parameters=[
            moveit_config.robot_description, 
            str(moveit_config.package_path / "config/ros2_controllers.yaml")],
        output="screen",
        condition=UnlessCondition("False"),
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition("True") and UnlessCondition("False"),
        executable="dashboard_client",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": "192.168.56.1"}],
    )
    
    controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition("False"),
        parameters=[
            {"headless_mode": True},
            {"joint_controller_active": False},
            {
                "consistent_controllers": [
                    "io_and_status_controller",
                    "force_torque_sensor_broadcaster",
                    "joint_state_broadcaster",
                    "speed_scaling_state_broadcaster",
                ]
            },
        ],
    )
    
    ld.add_action(ur_control_node)
    ld.add_action(dashboard_client_node)
    ld.add_action(controller_stopper_node)
    return ld
