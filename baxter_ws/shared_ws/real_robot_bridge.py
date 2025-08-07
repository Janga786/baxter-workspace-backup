#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import subprocess
import json
import time
import threading

class RealRobotBridge(Node):
    def __init__(self):
        super().__init__('real_robot_bridge')
        
        # ROS 2 publishers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to publish joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        # Real robot joint positions (will be updated from ROS 1)
        self.real_joint_positions = {
            'head_pan': 0.0,
            'left_s0': 0.0, 'left_s1': -0.55, 'left_e0': 0.0, 'left_e1': 0.75,
            'left_w0': 0.0, 'left_w1': 1.26, 'left_w2': 0.0,
            'right_s0': 0.0, 'right_s1': -0.55, 'right_e0': 0.0, 'right_e1': 0.75,
            'right_w0': 0.0, 'right_w1': 1.26, 'right_w2': 0.0
        }
        
        # Start thread to get real robot states
        self.bridge_thread = threading.Thread(target=self.get_real_robot_states, daemon=True)
        self.bridge_thread.start()
        
        self.get_logger().info('Real Robot Bridge started')
        self.get_logger().info('Bridging real Baxter joint states to ROS 2')
        
    def get_real_robot_states(self):
        """Get joint states from real robot via ROS 1"""
        while rclpy.ok():
            try:
                # Create script to get joint states from ROS 1
                script = '''
import rospy
from sensor_msgs.msg import JointState
import json
import sys

def joint_callback(msg):
    joint_data = {}
    for i, name in enumerate(msg.name):
        if i < len(msg.position):
            joint_data[name] = msg.position[i]
    print(json.dumps(joint_data))
    sys.stdout.flush()

rospy.init_node('joint_reader', anonymous=True)
rospy.Subscriber('/robot/joint_states', JointState, joint_callback)
rospy.sleep(1.0)  # Get one message
'''
                
                # Write script to temp file
                with open('/tmp/get_joints.py', 'w') as f:
                    f.write(script)
                
                # Execute in ROS 1 container
                result = subprocess.run([
                    'docker', 'run', '--rm', '--network', 'host',
                    '-v', '/tmp/get_joints.py:/tmp/get_joints.py',
                    '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                    'baxter_ros1', 'timeout', '3',
                    'bash', '-c',
                    'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/get_joints.py'
                ], capture_output=True, text=True, timeout=5)
                
                if result.stdout.strip():
                    try:
                        joint_data = json.loads(result.stdout.strip().split('\n')[-1])
                        # Update our positions with real data
                        for joint_name, position in joint_data.items():
                            if joint_name in self.real_joint_positions:
                                self.real_joint_positions[joint_name] = position
                        
                        self.get_logger().info('Updated joint states from real robot', once=True)
                    except json.JSONDecodeError:
                        pass
                        
            except Exception as e:
                self.get_logger().warn(f'Error getting real joint states: {e}', throttle_duration_sec=5.0)
                
            time.sleep(0.5)  # Update at 2Hz
        
    def publish_joint_states(self):
        """Publish current joint states to ROS 2"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Publish all joints
        msg.name = list(self.real_joint_positions.keys())
        msg.position = list(self.real_joint_positions.values())
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        
        self.joint_state_pub.publish(msg)
        
    def send_to_real_robot(self, joint_positions):
        """Send joint positions to real Baxter"""
        try:
            # Create command for real robot
            script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('moveit_bridge_cmd', anonymous=True)
pub = rospy.Publisher('/robot/limb/left/command_joint_position', JointState, queue_size=1)
rospy.sleep(1.0)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
msg.position = {joint_positions}

for _ in range(10):
    pub.publish(msg)
    rospy.sleep(0.1)
'''
            
            with open('/tmp/send_cmd.py', 'w') as f:
                f.write(script)
                
            subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', '/tmp/send_cmd.py:/tmp/send_cmd.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/send_cmd.py'
            ], check=True)
            
            self.get_logger().info('Command sent to real robot')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    bridge = RealRobotBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()