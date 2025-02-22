#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration  # Correct import for Duration

class JointControllerNode(Node):
    def __init__(self):
        super().__init__('joint_controller_node')
        # Create a publisher to send commands to the joint trajectory controller
        self.publisher = self.create_publisher(JointTrajectory, '/arm/joint_trajectory_controller/command', 10)

    def send_trajectory(self):
        # Create the JointTrajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']  # List of joints

        # Create a JointTrajectoryPoint message that specifies the desired joint positions
        point = JointTrajectoryPoint()
        point.positions = [1.0, 0.5, -0.5, 1.0, 0.5, -0.5]  # Example joint positions in radians (modify based on your robot)
        point.time_from_start = Duration(seconds=2.0)  # Correct way to create a Duration
        trajectory.points = [point]

        # Publish the trajectory to the joint trajectory controller
        self.publisher.publish(trajectory)
        self.get_logger().info("Sent joint trajectory")

def main(args=None):
    rclpy.init(args=args)
    node = JointControllerNode()
    node.send_trajectory()  # Send the trajectory command once
    rclpy.spin(node)  # Keep spinning to handle ROS callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
