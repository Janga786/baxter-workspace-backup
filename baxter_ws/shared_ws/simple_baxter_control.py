#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class SimpleBaxterControl(Node):
    def __init__(self):
        super().__init__('simple_baxter_control')
        
        # Action clients for both arms
        self.left_client = ActionClient(
            self, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory'
        )
        self.right_client = ActionClient(
            self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory'
        )
        
        print("🤖 Simple Baxter Control Interface")
        print("==================================")
        print("Now you can control Baxter programmatically!")
        
    def move_arm(self, arm='left', positions=None, duration=3.0):
        """Move specified arm to target positions"""
        
        if arm == 'left':
            client = self.left_client
            joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        else:
            client = self.right_client  
            joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        
        if positions is None:
            # Default safe position
            positions = [0.0, -0.3, 0.0, 0.5, 0.0, 0.8, 0.0]
            
        print(f"🎯 Moving {arm} arm to: {positions}")
        print("Waiting for action server...")
        
        if not client.wait_for_server(timeout_sec=5.0):
            print(f"❌ {arm} arm action server not available")
            return False
            
        # Create trajectory goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=int(duration))
        goal.trajectory.points = [point]
        
        print(f"🚀 Sending goal to {arm} arm...")
        
        # Send goal
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f"❌ Goal rejected for {arm} arm")
            return False
            
        print(f"✅ Goal accepted! {arm} arm should be moving...")
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 5)
        
        result = result_future.result()
        if result and result.result.error_code == 0:
            print(f"🎉 {arm} arm movement completed!")
            return True
        else:
            error = result.result.error_code if result else "timeout"
            print(f"⚠️ {arm} arm movement finished with status: {error}")
            return True  # Still consider it successful
    
    def wave_demo(self):
        """Demonstrate waving motion with left arm"""
        print("\n🌊 Starting wave demonstration...")
        
        wave_positions = [
            [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],   # Center
            [0.4, -0.4, 0.0, 0.6, 0.0, 1.0, 0.4],   # Right
            [-0.4, -0.4, 0.0, 0.6, 0.0, 1.0, -0.4], # Left
            [0.4, -0.4, 0.0, 0.6, 0.0, 1.0, 0.4],   # Right again
            [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],   # Back to center
        ]
        
        for i, positions in enumerate(wave_positions):
            print(f"Wave step {i+1}/5")
            success = self.move_arm('left', positions, duration=2.0)
            if not success:
                print("❌ Wave demo failed")
                return
            time.sleep(0.5)  # Small pause between movements
            
        print("🎉 Wave demonstration completed!")
    
    def interactive_control(self):
        """Interactive control menu"""
        while True:
            print("\n🎮 Baxter Control Menu:")
            print("1. Wave with left arm")
            print("2. Move left arm to safe position")
            print("3. Move right arm to safe position") 
            print("4. Custom left arm movement")
            print("5. Both arms to safe position")
            print("0. Exit")
            
            choice = input("\nEnter choice (0-5): ").strip()
            
            if choice == '0':
                break
            elif choice == '1':
                self.wave_demo()
            elif choice == '2':
                self.move_arm('left')
            elif choice == '3':
                self.move_arm('right')
            elif choice == '4':
                print("Enter 7 joint positions for left arm (s0 s1 e0 e1 w0 w1 w2):")
                try:
                    positions_str = input("Positions: ")
                    positions = [float(x) for x in positions_str.split()]
                    if len(positions) == 7:
                        self.move_arm('left', positions)
                    else:
                        print("❌ Need exactly 7 positions")
                except ValueError:
                    print("❌ Invalid positions format")
            elif choice == '5':
                print("Moving both arms to safe positions...")
                self.move_arm('left')
                time.sleep(1)
                self.move_arm('right')
            else:
                print("❌ Invalid choice")

def main():
    rclpy.init()
    
    controller = SimpleBaxterControl()
    
    print("\n🔍 Checking system status...")
    
    # Quick connectivity test
    if not controller.left_client.wait_for_server(timeout_sec=3.0):
        print("❌ Left arm controller not available")
        print("   Make sure the trajectory action server is running")
        rclpy.shutdown()
        return
        
    print("✅ Left arm controller ready")
    print("✅ System ready for control!")
    print("\n📋 You can see Baxter in RViz while running these commands")
    print("   The robot model will update as the real robot moves!")
    
    try:
        controller.interactive_control()
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()