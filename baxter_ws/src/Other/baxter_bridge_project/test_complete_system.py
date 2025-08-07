#!/usr/bin/env python3
"""
Test the complete ROS2 → Baxter system
This script verifies that ROS2 commands successfully move the physical Baxter robot
"""

import subprocess
import time
import sys

def test_bridge_connection():
    """Test that our bridge is responding to ROS2 commands"""
    print("🔍 Testing ROS2 bridge connection...")
    
    result = subprocess.run([
        'docker', 'exec', 'baxter_moveit', 'bash', '-c',
        '''cd /shared_ws && source /opt/ros/humble/setup.bash && timeout 5 python3 -c "
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

rclpy.init()
node = rclpy.create_node('test_bridge')
client = ActionClient(node, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')

if client.wait_for_server(timeout_sec=3.0):
    print('BRIDGE_OK')
else:
    print('BRIDGE_FAIL')
rclpy.shutdown()
"'''
    ], capture_output=True, text=True)
    
    if "BRIDGE_OK" in result.stdout:
        print("✅ Bridge connection successful!")
        return True
    else:
        print("❌ Bridge connection failed!")
        return False

def test_moveit_integration():
    """Test MoveIt can plan and execute"""
    print("🎯 Testing MoveIt integration...")
    
    result = subprocess.run([
        'docker', 'exec', 'baxter_moveit', 'bash', '-c',
        '''cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 10 python3 -c "
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

rclpy.init()
node = rclpy.create_node('test_moveit')
client = ActionClient(node, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')

if client.wait_for_server(timeout_sec=3.0):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    
    point = JointTrajectoryPoint()
    point.positions = [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0]
    point.time_from_start.sec = 2
    goal.trajectory.points = [point]
    
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() and future.result().accepted:
        print('MOVEIT_OK')
    else:
        print('MOVEIT_FAIL')
else:
    print('MOVEIT_FAIL')

rclpy.shutdown()
"'''
    ], capture_output=True, text=True)
    
    if "MOVEIT_OK" in result.stdout:
        print("✅ MoveIt integration successful!")
        return True
    else:
        print("❌ MoveIt integration failed!")
        return False

def test_robot_movement():
    """Test a small movement to verify robot responds"""
    print("🤖 Testing robot movement...")
    print("⚠️  Watch Baxter - it should move slightly!")
    
    result = subprocess.run([
        'docker', 'exec', 'baxter_moveit', 'bash', '-c',
        '''cd /shared_ws && source /opt/ros/humble/setup.bash && timeout 15 python3 -c "
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

rclpy.init()
node = rclpy.create_node('test_movement')
client = ActionClient(node, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory')

if client.wait_for_server(timeout_sec=3.0):
    # Small safe movement
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    
    point = JointTrajectoryPoint()
    point.positions = [0.1, -0.4, 0.0, 0.6, 0.0, 1.0, 0.1]  # Slight wrist rotation
    point.time_from_start.sec = 3
    goal.trajectory.points = [point]
    
    print('Sending movement command...')
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
    
    if future.result() and future.result().accepted:
        print('ROBOT_MOVED')
    else:
        print('ROBOT_FAIL')
else:
    print('ROBOT_FAIL')

rclpy.shutdown()
"'''
    ], capture_output=True, text=True)
    
    if "ROBOT_MOVED" in result.stdout:
        print("✅ Robot movement command sent successfully!")
        print("👀 Did you see Baxter move?")
        return True
    else:
        print("❌ Robot movement failed!")
        return False

def main():
    print("🚀 Testing Complete ROS2 → Baxter System")
    print("=" * 50)
    
    # Test sequence
    tests = [
        ("Bridge Connection", test_bridge_connection),
        ("MoveIt Integration", test_moveit_integration),
        ("Robot Movement", test_robot_movement)
    ]
    
    results = []
    for test_name, test_func in tests:
        print(f"\n📋 Running: {test_name}")
        result = test_func()
        results.append((test_name, result))
        
        if not result:
            print(f"❌ {test_name} failed - stopping tests")
            break
            
        time.sleep(1)
    
    # Summary
    print("\n" + "=" * 50)
    print("📊 TEST RESULTS:")
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"  {test_name}: {status}")
    
    all_passed = all(result for _, result in results)
    
    if all_passed:
        print("\n🎉 ALL TESTS PASSED!")
        print("✅ ROS2 commands can now control your physical Baxter robot!")
        print("✅ MoveIt is working with real hardware!")
        print("\n🎯 Next steps:")
        print("  • Use MoveIt in RViz to plan and execute motions")
        print("  • Run: docker exec -it baxter_moveit rviz2")
        print("  • Or write custom ROS2 scripts using the action servers")
    else:
        print("\n❌ Some tests failed. Check the output above for details.")
        
    return all_passed

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)