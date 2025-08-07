#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class TestROS2Control(Node):
    def __init__(self):
        super().__init__('test_ros2_control')
        
        # Action client for left arm
        self.client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/left_arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('🤖 Testing ROS 2 control of Baxter robot')
        
    def test_movement(self):
        """Test simple movement via ROS 2 action"""
        self.get_logger().info('Waiting for action server...')
        
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Action server not available')
            return False
            
        self.get_logger().info('✅ Action server found!')
        
        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = [
            'left_s0', 'left_s1', 'left_e0', 'left_e1', 
            'left_w0', 'left_w1', 'left_w2'
        ]
        
        # Define movement sequence
        movements = [
            # Wave sequence
            ([0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0], 2.0),   # Center
            ([0.4, -0.4, 0.0, 0.6, 0.0, 1.0, 0.4], 4.0),   # Right
            ([-0.4, -0.4, 0.0, 0.6, 0.0, 1.0, -0.4], 6.0), # Left
            ([0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0], 8.0),   # Center
        ]
        
        # Add trajectory points
        for positions, time_sec in movements:
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=int(time_sec))
            goal.trajectory.points.append(point)
        
        self.get_logger().info('🎯 Sending wave motion trajectory...')
        
        # Send goal
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            return False
            
        self.get_logger().info('✅ Goal accepted! Baxter should be waving...')
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=15.0)
        
        result = result_future.result()
        if result and result.result.error_code == 0:
            self.get_logger().info('🎉 Wave motion completed successfully!')
            self.get_logger().info('✅ ROS 2 successfully controlled physical Baxter!')
            return True
        else:
            error_code = result.result.error_code if result else "timeout"
            self.get_logger().error(f'❌ Movement failed: {error_code}')
            return False

def main():
    rclpy.init()
    
    test_node = TestROS2Control()
    
    print("🤖 ROS 2 Baxter Control Test")
    print("===========================")
    print("This will test if ROS 2 commands can control the physical Baxter robot.")
    print("The robot should perform a waving motion with its left arm.")
    print("")
    
    input("Press Enter to start test (make sure Baxter is enabled and ready)...")
    
    try:
        success = test_node.test_movement()
        
        if success:
            print("\n🎉 TEST PASSED!")
            print("✅ ROS 2 commands successfully control Baxter hardware")
            print("✅ Your integration is working perfectly!")
        else:
            print("\n❌ TEST FAILED!")
            print("Check the robot connection and try again")
            
    except KeyboardInterrupt:
        print("\nTest interrupted")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()