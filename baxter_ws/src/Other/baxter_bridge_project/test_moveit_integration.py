#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class MoveItIntegrationTest(Node):
    """Test MovEIt integration with real Baxter robot"""
    
    def __init__(self):
        super().__init__('moveit_integration_test')
        
        # Action clients for both arms
        self.left_arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory'
        )
        
        self.right_arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('🧪 MovEIt Integration Test Node Started')
        
    def test_left_arm_wave(self):
        """Test left arm wave motion through MovEIt action server"""
        self.get_logger().info('🌊 Testing left arm wave motion...')
        
        if not self.left_arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Left arm action server not available!')
            return False
        
        # Create wave trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'left_s0', 'left_s1', 'left_e0', 'left_e1',
            'left_w0', 'left_w1', 'left_w2'
        ]
        
        # Wave positions
        wave_positions = [
            [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],   # Start position
            [0.4, -0.4, 0.0, 0.6, 0.0, 1.0, 0.4],   # Wave right
            [-0.4, -0.4, 0.0, 0.6, 0.0, 1.0, -0.4], # Wave left
            [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],   # Back to center
        ]
        
        # Create trajectory points
        for i, positions in enumerate(wave_positions):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * len(positions)
            point.accelerations = [0.0] * len(positions)
            point.time_from_start = Duration(sec=2*(i+1), nanosec=0)
            goal.trajectory.points.append(point)
        
        self.get_logger().info('📤 Sending left arm wave trajectory...')
        future = self.left_arm_client.send_goal_async(goal)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is None:
            self.get_logger().error('❌ Failed to send goal to left arm')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Left arm goal was rejected')
            return False
        
        self.get_logger().info('✅ Left arm goal accepted, waiting for completion...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        if result_future.result() is None:
            self.get_logger().error('❌ Left arm trajectory execution timed out')
            return False
        
        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('🎉 Left arm wave completed successfully!')
            return True
        else:
            self.get_logger().error(f'❌ Left arm trajectory failed with error code: {result.error_code}')
            return False
    
    def test_right_arm_movement(self):
        """Test right arm movement through MovEIt action server"""
        self.get_logger().info('🤖 Testing right arm movement...')
        
        if not self.right_arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Right arm action server not available!')
            return False
        
        # Create simple movement trajectory
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'right_s0', 'right_s1', 'right_e0', 'right_e1',
            'right_w0', 'right_w1', 'right_w2'
        ]
        
        # Simple movement positions
        positions = [
            [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],   # Default position
            [0.3, -0.3, 0.0, 0.5, 0.0, 1.0, 0.0],      # Move position
            [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],   # Back to default
        ]
        
        # Create trajectory points
        for i, pos in enumerate(positions):
            point = JointTrajectoryPoint()
            point.positions = pos
            point.velocities = [0.0] * len(pos)
            point.accelerations = [0.0] * len(pos)
            point.time_from_start = Duration(sec=3*(i+1), nanosec=0)
            goal.trajectory.points.append(point)
        
        self.get_logger().info('📤 Sending right arm movement trajectory...')
        future = self.right_arm_client.send_goal_async(goal)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is None:
            self.get_logger().error('❌ Failed to send goal to right arm')
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Right arm goal was rejected')
            return False
        
        self.get_logger().info('✅ Right arm goal accepted, waiting for completion...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        if result_future.result() is None:
            self.get_logger().error('❌ Right arm trajectory execution timed out')
            return False
        
        result = result_future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('🎉 Right arm movement completed successfully!')
            return True
        else:
            self.get_logger().error(f'❌ Right arm trajectory failed with error code: {result.error_code}')
            return False
    
    def run_tests(self):
        """Run all integration tests"""
        self.get_logger().info('🚀 Starting MovEIt Integration Tests')
        self.get_logger().info('=' * 50)
        
        tests_passed = 0
        total_tests = 2
        
        # Test left arm
        if self.test_left_arm_wave():
            tests_passed += 1
        
        time.sleep(2)  # Brief pause between tests
        
        # Test right arm
        if self.test_right_arm_movement():
            tests_passed += 1
        
        # Results
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'📊 Test Results: {tests_passed}/{total_tests} passed')
        
        if tests_passed == total_tests:
            self.get_logger().info('🎉 ALL TESTS PASSED! MovEIt integration is working correctly.')
            self.get_logger().info('✅ Real robot movement through MovEIt is functional!')
        else:
            self.get_logger().error('❌ Some tests failed. Check the logs above for details.')
        
        return tests_passed == total_tests


def main(args=None):
    rclpy.init(args=args)
    
    test_node = MoveItIntegrationTest()
    
    try:
        # Run the tests
        success = test_node.run_tests()
        
        if success:
            print("\n🎯 INTEGRATION TEST SUCCESS!")
            print("MovEIt is properly integrated with real Baxter robot!")
        else:
            print("\n❌ INTEGRATION TEST FAILED!")
            print("Check the logs for specific error details.")
            
    except KeyboardInterrupt:
        test_node.get_logger().info('🛑 Test interrupted by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()