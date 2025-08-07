#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import subprocess
import time

class BaxterCommandBridge(Node):
    def __init__(self):
        super().__init__('baxter_command_bridge')
        
        # Subscribe to command topics from ROS2
        self.left_sub = self.create_subscription(
            JointState,
            '/baxter/left_arm/command',
            self.left_arm_callback,
            10
        )
        
        self.right_sub = self.create_subscription(
            JointState,
            '/baxter/right_arm/command', 
            self.right_arm_callback,
            10
        )
        
        self.get_logger().info('🌉 Baxter Command Bridge started')
        self.get_logger().info('Listening for commands on:')
        self.get_logger().info('  - /baxter/left_arm/command')
        self.get_logger().info('  - /baxter/right_arm/command')
        
    def left_arm_callback(self, msg):
        """Forward left arm commands to real Baxter"""
        self.send_to_baxter('left', msg)
        
    def right_arm_callback(self, msg):
        """Forward right arm commands to real Baxter"""
        self.send_to_baxter('right', msg)
        
    def send_to_baxter(self, arm, msg):
        """Send command to real Baxter robot"""
        try:
            positions = list(msg.position)
            
            script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('bridge_cmd', anonymous=True)
pub = rospy.Publisher('/robot/limb/{arm}/command_joint_position', JointState, queue_size=1)
rospy.sleep(1.0)

cmd = JointState()
cmd.header.stamp = rospy.Time.now()
cmd.name = ['{arm}_s0', '{arm}_s1', '{arm}_e0', '{arm}_e1', '{arm}_w0', '{arm}_w1', '{arm}_w2']
cmd.position = {positions}

for i in range(5):
    pub.publish(cmd)
    rospy.sleep(0.1)
'''
            
            # Write script
            with open(f'/tmp/bridge_cmd_{arm}.py', 'w') as f:
                f.write(script)
            
            # Execute command
            subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', f'/tmp/bridge_cmd_{arm}.py:/tmp/bridge_cmd_{arm}.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                f'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/bridge_cmd_{arm}.py'
            ], capture_output=True, timeout=5)
            
            self.get_logger().info(f'📡 Sent command to {arm} arm: {positions}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error sending to {arm} arm: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    bridge = BaxterCommandBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()