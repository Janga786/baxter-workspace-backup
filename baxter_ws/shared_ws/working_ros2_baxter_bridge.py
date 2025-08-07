#!/usr/bin/env python3
"""
Working ROS2 to Baxter Bridge
Uses proven ROS1 communication for actual robot movement
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import subprocess
import threading
import time

class WorkingBaxterBridge(Node):
    def __init__(self):
        super().__init__('working_baxter_bridge')
        
        # Action servers for MoveIt integration
        self.left_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            self.execute_left_trajectory
        )
        
        self.right_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            self.execute_right_trajectory
        )
        
        # Joint state publisher for MoveIt feedback
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Start joint state publisher thread
        self.joint_state_thread = threading.Thread(target=self.publish_joint_states, daemon=True)
        self.joint_state_thread.start()
        
        self.get_logger().info('🤖 Working Baxter Bridge started!')
        self.get_logger().info('✅ Left arm action server: /left_arm_controller/follow_joint_trajectory')
        self.get_logger().info('✅ Right arm action server: /right_arm_controller/follow_joint_trajectory')

    def execute_left_trajectory(self, goal_handle):
        """Execute trajectory on left arm using proven ROS1 commands"""
        return self.execute_trajectory(goal_handle, 'left')
    
    def execute_right_trajectory(self, goal_handle):
        """Execute trajectory on right arm using proven ROS1 commands"""
        return self.execute_trajectory(goal_handle, 'right')
    
    def execute_trajectory(self, goal_handle, arm):
        """Execute trajectory using our proven ROS1 movement system"""
        self.get_logger().info(f'🎯 Executing {arm} arm trajectory...')
        
        request = goal_handle.request
        trajectory = request.trajectory
        
        if not trajectory.points:
            self.get_logger().error('❌ Empty trajectory received')
            goal_handle.abort()
            return FollowJointTrajectory.Result()
        
        # Get the final point (where we want to end up)
        final_point = trajectory.points[-1]
        target_positions = list(final_point.positions)
        
        self.get_logger().info(f'🎯 Moving {arm} arm to: {target_positions}')
        
        # Accept the goal
        goal_handle.succeed()
        
        # Execute movement using our proven system
        success = self.send_to_baxter(arm, target_positions)
        
        if success:
            self.get_logger().info(f'✅ {arm} arm movement completed successfully!')
        else:
            self.get_logger().error(f'❌ {arm} arm movement failed!')
        
        # Return result
        result = FollowJointTrajectory.Result()
        result.error_code = 0 if success else -1
        return result
    
    def send_to_baxter(self, arm, positions):
        """Send movement command to Baxter using proven ROS1 system"""
        try:
            # Create ROS1 script for movement
            script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('bridge_mover', anonymous=True)
pub = rospy.Publisher('/robot/limb/{arm}/command_joint_position', JointState, queue_size=1)
rospy.sleep(1.0)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = ['{arm}_s0', '{arm}_s1', '{arm}_e0', '{arm}_e1', '{arm}_w0', '{arm}_w1', '{arm}_w2']
msg.position = {positions}

print("Sending movement command...")
for i in range(20):
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rospy.sleep(0.1)
    
print("Movement command sent!")
'''
            
            # Write script to temporary file
            script_path = f'/tmp/bridge_move_{arm}.py'
            with open(script_path, 'w') as f:
                f.write(script)
            
            # Execute via Docker
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', f'{script_path}:{script_path}',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                f'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python {script_path}'
            ], capture_output=True, text=True, timeout=15)
            
            if result.returncode == 0:
                self.get_logger().info(f'✅ {arm} arm ROS1 command successful')
                return True
            else:
                self.get_logger().error(f'❌ ROS1 command failed: {result.stderr}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error sending to Baxter: {e}')
            return False
    
    def publish_joint_states(self):
        """Publish joint states from Baxter for MoveIt feedback"""
        rate = self.create_rate(10)  # 10 Hz
        
        # Default joint positions (will be updated with real data)
        joint_names = [
            'left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2',
            'right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'
        ]
        
        while rclpy.ok():
            try:
                # Get real joint states from Baxter (simplified for now)
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = joint_names
                joint_state.position = [0.0] * len(joint_names)  # TODO: Get real positions
                joint_state.velocity = [0.0] * len(joint_names)
                joint_state.effort = [0.0] * len(joint_names)
                
                self.joint_state_pub.publish(joint_state)
                
            except Exception as e:
                self.get_logger().error(f'Error publishing joint states: {e}')
            
            rate.sleep()

def main():
    rclpy.init()
    
    bridge = WorkingBaxterBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()