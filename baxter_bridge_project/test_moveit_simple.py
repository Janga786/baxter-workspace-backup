#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander.roscpp_initializer import roscpp_initialize
import sys

def test_moveit():
    print("🎯 Testing MovEIt Integration")
    print("=" * 40)
    
    # Initialize ROS
    rclpy.init()
    roscpp_initialize(sys.argv)
    
    try:
        # Initialize MovEIt components
        print("1. Initializing MovEIt components...")
        robot = RobotCommander()
        scene = PlanningSceneInterface()
        
        print("✅ MovEIt initialized successfully")
        
        # Get robot information
        print(f"\n2. Robot Information:")
        print(f"   Robot name: {robot.get_robot_name()}")
        print(f"   Planning frame: {robot.get_planning_frame()}")
        
        # Get group names
        group_names = robot.get_group_names()
        print(f"   Available groups: {group_names}")
        
        # Test each arm group
        for group_name in group_names:
            if 'arm' in group_name.lower():
                print(f"\n3. Testing {group_name} group:")
                try:
                    group = robot.get_group(group_name)
                    print(f"   End effector: {group.get_end_effector_link()}")
                    print(f"   Joints: {group.get_active_joints()}")
                    print(f"   Current pose: {group.get_current_pose()}")
                    print(f"   ✅ {group_name} group accessible")
                except Exception as e:
                    print(f"   ❌ Error with {group_name}: {e}")
        
        print(f"\n🎉 MovEIt integration test completed!")
        return True
        
    except Exception as e:
        print(f"❌ MovEIt test failed: {e}")
        return False
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    test_moveit()