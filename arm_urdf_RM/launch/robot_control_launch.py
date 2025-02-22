from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch ros2_control node and pass the configuration
        Node(
            package='ros2_control',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=['/home/souri/test_ws/src/arm_urdf/config/controllers.yaml'],
            remappings=[('/joint_states', '/robot/joint_states')]
        ),
        # Spawn the joint_state_controller
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['joint_state_controller'],
            output='screen'
        ),
        # Spawn the joint_trajectory_controller
        Node(
            package='controller_manager',
            executable='spawner.py',
            arguments=['joint_trajectory_controller'],
            output='screen'
        ),
    ])
