# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import generate_demo_launch


# def generate_launch_description():
#     moveit_config = MoveItConfigsBuilder("gen3_lite_gen3_lite_2f", package_name="rcup_moveit_config").to_moveit_configs()
#     return generate_demo_launch(moveit_config)
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Build MoveIt configuration using MoveItConfigsBuilder
    moveit_config = MoveItConfigsBuilder(
        robot_name="gen3_lite_gen3_lite_2f",  # Robot name
        package_name="rcup_moveit_config"     # Package name
    ).to_moveit_configs()

    # Verify that the RViz config path is set manually
    rviz_config_path = "/home/soham/ws_moveit2/src/rcup_moveit_config/config/moveit.rviz"

    # Start RViz2 with MoveIt configuration
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],  # Explicitly provide the RViz config file path
        output="screen",
        parameters=[
            {"robot_description": moveit_config.robot_description},
            {"robot_description_semantic": moveit_config.robot_description_semantic},
        ]
    )

    return LaunchDescription([rviz_node])
