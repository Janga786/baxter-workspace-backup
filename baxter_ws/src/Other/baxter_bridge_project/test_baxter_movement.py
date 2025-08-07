#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import moveit_commander
import geometry_msgs.msg
import sys
import time


class BaxterMovementTest(Node):
    def __init__(self):
        super().__init__('baxter_movement_test')
        
        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize robot and scene
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Initialize planning groups
        try:
            self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
            self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
            self.get_logger().info("MoveIt planning groups initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize planning groups: {e}")
            return
        
        # Set planning parameters
        self.left_arm_group.set_planning_time(10.0)
        self.right_arm_group.set_planning_time(10.0)
        self.left_arm_group.set_num_planning_attempts(3)
        self.right_arm_group.set_num_planning_attempts(3)
        
        self.get_logger().info("Baxter Movement Test Node Initialized")
        
    def test_arm_movement(self):
        """Test basic arm movement"""
        try:
            # Get current joint values
            current_left = self.left_arm_group.get_current_joint_values()
            current_right = self.right_arm_group.get_current_joint_values()
            
            self.get_logger().info(f"Current left arm joints: {current_left}")
            self.get_logger().info(f"Current right arm joints: {current_right}")
            
            # Plan to home position for left arm
            self.get_logger().info("Moving left arm to home position...")
            self.left_arm_group.set_named_target("left_arm_home")
            plan_left = self.left_arm_group.plan()
            
            if plan_left[0]:  # ROS 2 returns tuple (success, plan, time, error)
                self.get_logger().info("Left arm plan successful, executing...")
                self.left_arm_group.execute(plan_left[1], wait=True)
                self.get_logger().info("Left arm moved to home position")
            else:
                self.get_logger().warn("Left arm planning failed")
            
            time.sleep(1.0)
            
            # Plan to home position for right arm
            self.get_logger().info("Moving right arm to home position...")
            self.right_arm_group.set_named_target("right_arm_home")
            plan_right = self.right_arm_group.plan()
            
            if plan_right[0]:
                self.get_logger().info("Right arm plan successful, executing...")
                self.right_arm_group.execute(plan_right[1], wait=True)
                self.get_logger().info("Right arm moved to home position")
            else:
                self.get_logger().warn("Right arm planning failed")
                
            time.sleep(2.0)
            
            # Perform a simple wave motion with left arm
            self.wave_left_arm()
            
        except Exception as e:
            self.get_logger().error(f"Movement test failed: {e}")
    
    def wave_left_arm(self):
        """Make Baxter wave with left arm"""
        try:
            self.get_logger().info("Starting wave motion with left arm...")
            
            # Wave motion - modify shoulder and wrist joints
            wave_positions = [
                # Joint order: [s0, s1, e0, e1, w0, w1, w2]
                [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],    # Home
                [0.3, -0.55, 0.0, 0.75, 0.0, 1.26, 0.5],    # Wave right
                [-0.3, -0.55, 0.0, 0.75, 0.0, 1.26, -0.5],  # Wave left
                [0.3, -0.55, 0.0, 0.75, 0.0, 1.26, 0.5],    # Wave right
                [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],    # Home
            ]
            
            for i, position in enumerate(wave_positions):
                self.get_logger().info(f"Wave motion step {i+1}/5")
                self.left_arm_group.set_joint_value_target(position)
                plan = self.left_arm_group.plan()
                
                if plan[0]:
                    self.left_arm_group.execute(plan[1], wait=True)
                    time.sleep(0.5)
                else:
                    self.get_logger().warn(f"Wave step {i+1} planning failed")
                    
            self.get_logger().info("Wave motion completed!")
            
        except Exception as e:
            self.get_logger().error(f"Wave motion failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        test_node = BaxterMovementTest()
        
        # Wait a moment for initialization
        time.sleep(2.0)
        
        # Run the movement test
        test_node.test_arm_movement()
        
        # Keep the node running
        rclpy.spin(test_node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        moveit_commander.roscpp_shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()