#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import subprocess
import time

class BridgeTest(Node):
    def __init__(self):
        super().__init__('bridge_test')
        
        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.latest_joint_states = None
        self.get_logger().info('Bridge Test Node started')
        
    def joint_state_callback(self, msg):
        self.latest_joint_states = msg
        
    def test_bridge(self):
        """Test that the bridge is working"""
        print("🔍 Testing ROS 1 to ROS 2 Bridge...")
        
        # Wait for joint states
        timeout = time.time() + 5
        while self.latest_joint_states is None and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.latest_joint_states is None:
            print("❌ No joint states received in ROS 2")
            return False
            
        print("✅ Joint states are flowing from ROS 1 to ROS 2")
        print(f"   Received {len(self.latest_joint_states.name)} joints")
        
        # Show current positions
        print("📊 Current joint positions:")
        for i, name in enumerate(self.latest_joint_states.name):
            if i < len(self.latest_joint_states.position):
                print(f"   {name}: {self.latest_joint_states.position[i]:.3f}")
                
        return True
        
    def test_robot_movement(self):
        """Test sending commands to real robot"""
        print("\n🤖 Testing Robot Movement...")
        
        # Define a simple wave motion
        positions_sequence = [
            [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],  # Start
            [0.3, -0.4, 0.0, 0.6, 0.0, 1.0, 0.3],  # Right
            [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],  # Back to start
        ]
        
        for i, positions in enumerate(positions_sequence):
            print(f"Moving to position {i+1}/3...")
            
            success = self.send_to_real_robot(positions)
            if success:
                print(f"✅ Position {i+1} sent successfully")
            else:
                print(f"❌ Failed to send position {i+1}")
                return False
                
            time.sleep(2.0)
            
        print("✅ Movement test completed!")
        return True
        
    def send_to_real_robot(self, positions):
        """Send joint positions to real robot"""
        try:
            script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('bridge_test_cmd', anonymous=True)
pub = rospy.Publisher('/robot/limb/left/command_joint_position', JointState, queue_size=1)
rospy.sleep(1.0)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
msg.position = {positions}

for _ in range(10):
    pub.publish(msg)
    rospy.sleep(0.1)
'''
            
            with open('/tmp/bridge_test_cmd.py', 'w') as f:
                f.write(script)
                
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', '/tmp/bridge_test_cmd.py:/tmp/bridge_test_cmd.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/bridge_test_cmd.py'
            ], capture_output=True, text=True, timeout=10)
            
            return result.returncode == 0
            
        except Exception as e:
            print(f"Error: {e}")
            return False

def main():
    print("🧪 Baxter Bridge and Movement Test")
    print("==================================")
    
    rclpy.init()
    test_node = BridgeTest()
    
    try:
        # Test 1: Bridge functionality
        if not test_node.test_bridge():
            print("❌ Bridge test failed")
            return
            
        # Test 2: Robot movement
        print("\nTesting robot movement...")
        if test_node.test_robot_movement():
            print("✅ All tests passed!")
            print("\n🎉 Your system is working:")
            print("   - ROS 1 to ROS 2 bridge: ✅")
            print("   - Joint state publishing: ✅") 
            print("   - Robot movement commands: ✅")
            print("   - RViz should show the robot now!")
        else:
            print("❌ Robot movement test failed")
            
    except KeyboardInterrupt:
        print("Test interrupted")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()