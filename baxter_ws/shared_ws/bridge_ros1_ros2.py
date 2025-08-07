#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import subprocess
import time
import threading

class BaxterROS1ROS2Bridge(Node):
    def __init__(self):
        super().__init__('baxter_ros1_ros2_bridge')
        
        # ROS 2 publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to publish joint states for MoveIt
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Current joint positions (will be updated from real robot)
        self.current_positions = {
            'head_pan': 0.0,
            'left_s0': 0.0, 'left_s1': -0.55, 'left_e0': 0.0, 'left_e1': 0.75,
            'left_w0': 0.0, 'left_w1': 1.26, 'left_w2': 0.0,
            'right_s0': 0.0, 'right_s1': -0.55, 'right_e0': 0.0, 'right_e1': 0.75,
            'right_w0': 0.0, 'right_w1': 1.26, 'right_w2': 0.0
        }
        
        self.get_logger().info('Baxter ROS1-ROS2 Bridge started')
        self.get_logger().info('Publishing joint states for MoveIt visualization')
        
    def publish_joint_states(self):
        """Publish joint states for MoveIt"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        msg.name = list(self.current_positions.keys())
        msg.position = list(self.current_positions.values())
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    bridge = BaxterROS1ROS2Bridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()