from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5e_workcell", package_name="ur5e_cell_msa_config")        
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        ).to_moveit_configs()
    )
    return generate_move_group_launch(moveit_config)
