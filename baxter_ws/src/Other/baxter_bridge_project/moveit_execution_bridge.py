#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import subprocess
import time

class MoveItExecutionBridge(Node):
    def __init__(self):
        super().__init__('moveit_execution_bridge')
        
        # Action client for MoveIt execution
        self._action_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        
        # Subscribe to joint states
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Store current joint states
        self.current_joint_states = {}
        
        self.get_logger().info('MoveIt Execution Bridge started')
        self.get_logger().info('This bridge will execute MoveIt trajectories on real Baxter')
        
    def joint_state_callback(self, msg):
        """Update current joint states"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
                
    def execute_on_real_baxter(self, trajectory):
        """Execute trajectory on real Baxter using ROS 1 commands"""
        self.get_logger().info(f'Executing trajectory with {len(trajectory.points)} points')
        
        # Extract left arm joints
        left_arm_joints = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
        
        for point in trajectory.points:
            # Create command for this trajectory point
            joint_positions = []
            for joint_name in left_arm_joints:
                if joint_name in trajectory.joint_names:
                    idx = trajectory.joint_names.index(joint_name)
                    if idx < len(point.positions):
                        joint_positions.append(point.positions[idx])
                    else:
                        joint_positions.append(0.0)
                else:
                    joint_positions.append(0.0)
            
            # Send command to real robot
            self.send_baxter_command(joint_positions)
            
            # Wait for execution (simplified timing)
            time.sleep(2.0)
            
        self.get_logger().info('Trajectory execution completed')
        
    def send_baxter_command(self, positions):
        """Send joint position command to real Baxter"""
        try:
            # Create a simple Python script to send the command
            cmd_script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('temp_commander', anonymous=True)
pub = rospy.Publisher('/robot/limb/left/command_joint_position', JointState, queue_size=1)
rospy.sleep(1.0)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
msg.position = {positions}

for _ in range(5):
    pub.publish(msg)
    rospy.sleep(0.1)
'''
            
            # Write to temp file and execute via ROS 1 container
            with open('/tmp/baxter_cmd.py', 'w') as f:
                f.write(cmd_script)
                
            # Execute via Docker
            subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', '/tmp/baxter_cmd.py:/tmp/baxter_cmd.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/baxter_cmd.py'
            ], check=True, capture_output=True)
            
            self.get_logger().info(f'Sent positions: {positions}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')

def main(args=None):
    rclpy.init(args=args)
    bridge = MoveItExecutionBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()