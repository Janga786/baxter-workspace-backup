#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import moveit_commander
import sys
import time
import subprocess

class MoveItExecutionTest(Node):
    def __init__(self):
        super().__init__('moveit_execution_test')
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
        
        self.get_logger().info("MoveIt Execution Test initialized")
        
    def test_simple_movement(self):
        """Test simple movement with real execution"""
        self.get_logger().info("=== Testing MoveIt to Real Robot Execution ===")
        
        # Get current position
        current = self.left_arm_group.get_current_joint_values()
        self.get_logger().info(f"Current position: {current}")
        
        # Define a simple target (small movement)
        target = current.copy()
        target[0] += 0.2  # Move shoulder joint slightly
        
        self.get_logger().info(f"Target position: {target}")
        
        # Set target
        self.left_arm_group.set_joint_value_target(target)
        
        # Plan
        self.get_logger().info("Planning movement...")
        plan = self.left_arm_group.plan()
        
        if plan[0]:
            self.get_logger().info("✅ Planning successful!")
            
            # Execute directly on real robot
            self.get_logger().info("🤖 Executing on real Baxter...")
            
            # Get final position from plan
            final_point = plan[1].joint_trajectory.points[-1]
            positions = list(final_point.positions)
            
            # Send to real robot
            self.send_to_real_baxter(positions)
            
            # Wait and return to original position
            time.sleep(3.0)
            
            self.get_logger().info("🔄 Returning to original position...")
            self.send_to_real_baxter(current)
            
            self.get_logger().info("✅ Test completed!")
            
        else:
            self.get_logger().error("❌ Planning failed!")
            
    def send_to_real_baxter(self, positions):
        """Send positions to real Baxter"""
        try:
            script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('test_executor', anonymous=True)
pub = rospy.Publisher('/robot/limb/left/command_joint_position', JointState, queue_size=1)
rospy.sleep(1.0)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
msg.position = {positions}

print("Sending command to real robot...")
for i in range(15):
    pub.publish(msg)
    rospy.sleep(0.1)
print("Command sent!")
'''
            
            with open('/tmp/test_cmd.py', 'w') as f:
                f.write(script)
                
            self.get_logger().info("📡 Sending command to real robot...")
            
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', '/tmp/test_cmd.py:/tmp/test_cmd.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/test_cmd.py'
            ], capture_output=True, text=True)
            
            if result.returncode == 0:
                self.get_logger().info("✅ Command sent successfully!")
                if result.stdout:
                    self.get_logger().info(f"Output: {result.stdout.strip()}")
            else:
                self.get_logger().error(f"❌ Command failed: {result.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    test = MoveItExecutionTest()
    
    print("🤖 MoveIt Real Robot Execution Test")
    print("==================================")
    print("This will:")
    print("1. Plan a small movement with MoveIt")
    print("2. Execute on the real Baxter robot")
    print("3. Return to original position")
    print("")
    input("Press Enter to start test...")
    
    try:
        # Wait for initialization
        time.sleep(2.0)
        
        # Run test
        test.test_simple_movement()
        
    except KeyboardInterrupt:
        print("Test interrupted")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        moveit_commander.roscpp_shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()