#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
import tf2_ros
import time
from urdf_parser_py.urdf import URDF

class RobotPublisher(Node):
    def __init__(self):
        super().__init__('robot_publisher')
        
        # Create publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Create a TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Load the URDF from file (path to your URDF file)
        self.robot = URDF.from_xml_file('/home/souri/test_ws/src/arm_urdf/urdf/arm_urdf.urdf')  # Update with your URDF path
        self.get_logger().info(f"Loaded robot: {self.robot.name}")

        # Timer to periodically publish joint states and TF
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def timer_callback(self):
        # Publish joint states (example: 5 joints, replace with actual joint data)
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0]  # Initialize positions (set actual joint positions)
        joint_state.velocity = []
        joint_state.effort = []
        
        self.joint_state_pub.publish(joint_state)
        
        # Create a transform for the robot's base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base_link'
        
        # Set translation (position) and rotation (orientation) of the robot's base link
        transform.transform.translation.x = 1.0  # Example translation
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0  # No rotation (identity quaternion)
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    robot_publisher = RobotPublisher()
    rclpy.spin(robot_publisher)
    robot_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
