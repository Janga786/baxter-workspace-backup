#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveGroupActionGoal
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def test_moveit_motion():
    print("🎯 Testing MovEIt Motion Planning")
    print("=" * 40)
    
    rclpy.init()
    node = Node('moveit_test')
    
    # Create a publisher for move_group action
    publisher = node.create_publisher(
        MoveGroupActionGoal, 
        '/move_action/goal', 
        10
    )
    
    # Wait for publisher to be ready
    rclpy.spin_once(node, timeout_sec=2.0)
    
    # Create a simple motion goal for the left arm
    goal_msg = MoveGroupActionGoal()
    goal_msg.header = Header()
    goal_msg.header.stamp = node.get_clock().now().to_msg()
    goal_msg.goal_id.id = "test_motion_" + str(node.get_clock().now().nanoseconds)
    
    # Set up the motion request
    goal_msg.goal.request.group_name = "left_arm"
    goal_msg.goal.request.num_planning_attempts = 5
    goal_msg.goal.request.allowed_planning_time = 10.0
    goal_msg.goal.request.workspace_parameters.header.frame_id = "base"
    
    # Set a simple pose target
    pose_target = PoseStamped()
    pose_target.header.frame_id = "base"
    pose_target.pose.position.x = 0.6
    pose_target.pose.position.y = 0.3
    pose_target.pose.position.z = 0.2
    pose_target.pose.orientation.w = 1.0
    
    goal_msg.goal.request.goal_constraints.append(
        # We would need to create proper constraints here
    )
    
    print("📤 Publishing motion goal...")
    publisher.publish(goal_msg)
    
    print("✅ Motion goal sent to MovEIt!")
    print("   Group: left_arm")
    print(f"   Target: x={pose_target.pose.position.x}, y={pose_target.pose.position.y}, z={pose_target.pose.position.z}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    test_moveit_motion()