#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import time
import math

class BaxterMover(Node):
    def __init__(self):
        super().__init__('baxter_mover')
        
        print("🤖 SCRIPT 2: Programming Baxter Movement with MovEIt Integration")
        print("============================================================")
        
        # Action clients for both arms
        self.left_arm_client = ActionClient(
            self, FollowJointTrajectory, '/left_arm_controller/follow_joint_trajectory'
        )
        self.right_arm_client = ActionClient(
            self, FollowJointTrajectory, '/right_arm_controller/follow_joint_trajectory'
        )
        
        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        self.current_joint_states = {}
        self.ready = False
        
        print("Initializing Baxter movement system...")
        
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(msg.name):
            self.current_joint_states[name] = msg.position[i]
        if not self.ready and len(self.current_joint_states) > 10:
            self.ready = True
            print("✅ Joint states received - Baxter is ready!")
    
    def wait_for_servers(self):
        """Wait for action servers to be available"""
        print("🔄 Waiting for action servers...")
        
        left_ready = self.left_arm_client.wait_for_server(timeout_sec=10.0)
        right_ready = self.right_arm_client.wait_for_server(timeout_sec=10.0)
        
        if left_ready and right_ready:
            print("✅ Both arm action servers ready!")
            return True
        else:
            print("❌ Action servers not available")
            return False
    
    def create_trajectory(self, arm, target_positions, duration=5.0):
        """Create a joint trajectory for the specified arm"""
        goal_msg = FollowJointTrajectory.Goal()
        
        if arm == 'left':
            joint_names = [
                'left_s0', 'left_s1', 'left_e0', 'left_e1', 
                'left_w0', 'left_w1', 'left_w2'
            ]
        else:  # right
            joint_names = [
                'right_s0', 'right_s1', 'right_e0', 'right_e1',
                'right_w0', 'right_w1', 'right_w2'
            ]
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        
        # Get current positions as starting point
        current_positions = []
        for joint in joint_names:
            if joint in self.current_joint_states:
                current_positions.append(self.current_joint_states[joint])
            else:
                current_positions.append(0.0)
        
        # Add starting point
        start_point = JointTrajectoryPoint()
        start_point.positions = current_positions
        start_point.time_from_start.sec = 0
        start_point.time_from_start.nanosec = 0
        trajectory.points.append(start_point)
        
        # Add target point
        target_point = JointTrajectoryPoint()
        target_point.positions = target_positions
        target_point.time_from_start.sec = int(duration)
        target_point.time_from_start.nanosec = int((duration % 1) * 1e9)
        trajectory.points.append(target_point)
        
        goal_msg.trajectory = trajectory
        return goal_msg
    
    def move_arm(self, arm, positions, duration=5.0):
        """Move the specified arm to target positions"""
        print(f"🎯 Moving {arm} arm...")
        
        client = self.left_arm_client if arm == 'left' else self.right_arm_client
        goal = self.create_trajectory(arm, positions, duration)
        
        # Send goal
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(f"❌ {arm.title()} arm goal rejected")
            return False
        
        print(f"✅ {arm.title()} arm goal accepted - executing movement...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result()
        if result.result.error_code == 0:
            print(f"🎉 {arm.title()} arm movement completed successfully!")
            return True
        else:
            print(f"❌ {arm.title()} arm movement failed with error: {result.result.error_code}")
            return False
    
    def demo_movements(self):
        """Demonstrate various Baxter movements"""
        print("\n🎭 Starting Baxter Movement Demo")
        print("================================")
        
        # Demo 1: Neutral position
        print("\n1️⃣ Moving to neutral position...")
        neutral_left = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
        neutral_right = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
        
        success_left = self.move_arm('left', neutral_left, 3.0)
        time.sleep(1)
        success_right = self.move_arm('right', neutral_right, 3.0)
        
        if success_left and success_right:
            print("✅ Neutral position achieved!")
        
        time.sleep(2)
        
        # Demo 2: Wave with left arm
        print("\n2️⃣ Left arm waving motion...")
        wave_positions = [
            [0.5, -0.3, 0.0, 1.0, 0.0, 1.5, 0.0],   # Up
            [0.5, -0.3, 0.0, 1.0, 0.0, 0.5, 0.0],   # Down
            [0.5, -0.3, 0.0, 1.0, 0.0, 1.5, 0.0],   # Up
            [0.5, -0.3, 0.0, 1.0, 0.0, 1.0, 0.0],   # Middle
        ]
        
        for i, pos in enumerate(wave_positions):
            print(f"   Wave motion {i+1}/4...")
            self.move_arm('left', pos, 1.5)
            time.sleep(0.5)
        
        # Demo 3: Reach forward
        print("\n3️⃣ Reaching forward motion...")
        reach_left = [0.3, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0]
        reach_right = [-0.3, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0]
        
        self.move_arm('left', reach_left, 3.0)
        time.sleep(1)
        self.move_arm('right', reach_right, 3.0)
        
        time.sleep(2)
        
        # Return to neutral
        print("\n4️⃣ Returning to neutral...")
        self.move_arm('left', neutral_left, 3.0)
        time.sleep(1)
        self.move_arm('right', neutral_right, 3.0)
        
        print("\n🎉 Demo completed successfully!")
    
    def interactive_mode(self):
        """Interactive mode for custom movements"""
        print("\n🎮 Interactive Movement Mode")
        print("===========================")
        print("Available commands:")
        print("  'left neutral' - Move left arm to neutral")
        print("  'right neutral' - Move right arm to neutral")
        print("  'both neutral' - Move both arms to neutral")
        print("  'wave' - Wave with left arm")
        print("  'reach' - Reach forward with both arms")
        print("  'demo' - Run full demo")
        print("  'quit' - Exit")
        
        while True:
            try:
                command = input("\n🤖 Enter command: ").lower().strip()
                
                if command == 'quit':
                    break
                elif command == 'left neutral':
                    self.move_arm('left', [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0], 3.0)
                elif command == 'right neutral':
                    self.move_arm('right', [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0], 3.0)
                elif command == 'both neutral':
                    self.move_arm('left', [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0], 3.0)
                    time.sleep(1)
                    self.move_arm('right', [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0], 3.0)
                elif command == 'wave':
                    wave_pos = [0.5, -0.3, 0.0, 1.0, 0.0, 1.5, 0.0]
                    self.move_arm('left', wave_pos, 2.0)
                elif command == 'reach':
                    self.move_arm('left', [0.3, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0], 3.0)
                    time.sleep(1)
                    self.move_arm('right', [-0.3, 0.0, 0.0, 0.5, 0.0, 1.0, 0.0], 3.0)
                elif command == 'demo':
                    self.demo_movements()
                else:
                    print("❌ Unknown command. Try 'demo', 'wave', 'reach', or 'quit'")
                    
            except KeyboardInterrupt:
                break
        
        print("\n👋 Exiting interactive mode...")

def main():
    rclpy.init()
    
    try:
        mover = BaxterMover()
        
        # Wait for joint states
        print("⏳ Waiting for joint states...")
        while not mover.ready:
            rclpy.spin_once(mover, timeout_sec=0.1)
        
        # Wait for action servers
        if not mover.wait_for_servers():
            print("❌ Cannot connect to action servers. Make sure MovEIt is running!")
            return
        
        print("\n🎉 Baxter is ready for movement!")
        print("\nChoose an option:")
        print("1. Run demo movements")
        print("2. Interactive mode")
        
        try:
            choice = input("\nEnter choice (1 or 2): ").strip()
            
            if choice == '1':
                mover.demo_movements()
            elif choice == '2':
                mover.interactive_mode()
            else:
                print("Running demo by default...")
                mover.demo_movements()
                
        except KeyboardInterrupt:
            print("\n🛑 Movement interrupted by user")
        
        print("\n✅ Baxter movement script completed!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()