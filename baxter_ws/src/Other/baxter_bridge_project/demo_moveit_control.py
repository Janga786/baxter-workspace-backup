#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import moveit_commander
import geometry_msgs.msg
import sys
import time
import subprocess

class BaxterMoveItDemo(Node):
    def __init__(self):
        super().__init__('baxter_moveit_demo')
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Planning groups
        self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
        
        # Set planning parameters
        self.left_arm_group.set_planning_time(10.0)
        self.left_arm_group.set_num_planning_attempts(3)
        
        self.get_logger().info("Baxter MoveIt Demo initialized")
        self.get_logger().info("Available planning groups: {}".format(self.robot.get_group_names()))
        
    def demo_joint_movements(self):
        """Demonstrate joint space movements"""
        self.get_logger().info("=== Starting Joint Space Demo ===")
        
        # Define some demo positions
        positions = [
            # Home position
            {
                'left_s0': 0.0, 'left_s1': -0.55, 'left_e0': 0.0, 'left_e1': 0.75,
                'left_w0': 0.0, 'left_w1': 1.26, 'left_w2': 0.0
            },
            # Wave right
            {
                'left_s0': 0.5, 'left_s1': -0.55, 'left_e0': 0.0, 'left_e1': 0.75,
                'left_w0': 0.0, 'left_w1': 1.26, 'left_w2': 0.5
            },
            # Wave left
            {
                'left_s0': -0.5, 'left_s1': -0.55, 'left_e0': 0.0, 'left_e1': 0.75,
                'left_w0': 0.0, 'left_w1': 1.26, 'left_w2': -0.5
            }
        ]
        
        for i, position in enumerate(positions):
            self.get_logger().info(f"Moving to position {i+1}/3")
            
            # Set target
            self.left_arm_group.set_joint_value_target(position)
            
            # Plan movement
            plan = self.left_arm_group.plan()
            
            if plan[0]:  # If planning succeeded
                self.get_logger().info("Planning successful, executing...")
                
                # Execute on real robot
                self.execute_on_real_baxter(plan[1])
                
                # Wait between movements
                time.sleep(2.0)
            else:
                self.get_logger().error("Planning failed!")
                
    def execute_on_real_baxter(self, trajectory):
        """Execute MoveIt trajectory on real Baxter"""
        self.get_logger().info("Executing trajectory on real Baxter...")
        
        # Get trajectory points
        joint_trajectory = trajectory.joint_trajectory
        
        # Extract left arm joints
        left_arm_joints = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        
        # Execute each point
        for i, point in enumerate(joint_trajectory.points):
            self.get_logger().info(f"Executing point {i+1}/{len(joint_trajectory.points)}")
            
            # Create position list for left arm
            positions = []
            for joint_name in left_arm_joints:
                if joint_name in joint_trajectory.joint_names:
                    idx = joint_trajectory.joint_names.index(joint_name)
                    positions.append(point.positions[idx])
                else:
                    positions.append(0.0)
            
            # Send to real robot via ROS 1
            self.send_to_real_robot(positions)
            
            # Wait for execution
            time.sleep(1.0)
            
    def send_to_real_robot(self, positions):
        """Send joint positions to real Baxter via ROS 1"""
        try:
            # Create command script
            cmd_script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('moveit_executor', anonymous=True)
pub = rospy.Publisher('/robot/limb/left/command_joint_position', JointState, queue_size=1)
rospy.sleep(0.5)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
msg.position = {positions}

for _ in range(10):
    pub.publish(msg)
    rospy.sleep(0.1)
'''
            
            # Write to temp file
            with open('/tmp/moveit_cmd.py', 'w') as f:
                f.write(cmd_script)
                
            # Execute via ROS 1 container
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', '/tmp/moveit_cmd.py:/tmp/moveit_cmd.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/moveit_cmd.py'
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info("✅ Command sent to real robot")
            else:
                self.get_logger().error(f"❌ Failed to send command: {result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error sending to robot: {e}")

def main(args=None):
    print("🤖 Starting Baxter MoveIt Demo")
    print("==============================")
    
    rclpy.init(args=args)
    
    try:
        demo = BaxterMoveItDemo()
        
        # Wait a moment for initialization
        time.sleep(2.0)
        
        print("📋 Demo will perform:")
        print("1. Plan movements using MoveIt")
        print("2. Execute on real Baxter robot")
        print("3. Show joint space control")
        print("")
        input("Press Enter to start demo...")
        
        # Run the demo
        demo.demo_joint_movements()
        
        print("✅ Demo completed!")
        
    except KeyboardInterrupt:
        print("Demo interrupted")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        moveit_commander.roscpp_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()