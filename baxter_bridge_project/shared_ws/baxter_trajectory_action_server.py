#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
import subprocess
import time
import threading


class BaxterTrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('baxter_trajectory_action_server')
        
        # Action servers for both arms
        self.left_arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            self.execute_left_arm_trajectory
        )
        
        self.right_arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory',
            self.execute_right_arm_trajectory
        )
        
        # Subscribe to joint states for feedback
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Current joint states for feedback
        self.current_joint_states = {}
        self.feedback_timer = None
        
        self.get_logger().info('Baxter Trajectory Action Server started')
        self.get_logger().info('Listening for trajectory goals on:')
        self.get_logger().info('  - /left_arm_controller/follow_joint_trajectory')
        self.get_logger().info('  - /right_arm_controller/follow_joint_trajectory')
        
    def joint_state_callback(self, msg):
        """Update current joint states for feedback"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
    
    async def execute_left_arm_trajectory(self, goal_handle):
        """Execute trajectory on left arm"""
        return await self.execute_trajectory(goal_handle, 'left')
    
    async def execute_right_arm_trajectory(self, goal_handle):
        """Execute trajectory on right arm"""  
        return await self.execute_trajectory(goal_handle, 'right')
    
    async def execute_trajectory(self, goal_handle, arm):
        """Execute trajectory on specified arm"""
        self.get_logger().info(f'🎯 Executing {arm} arm trajectory with {len(goal_handle.request.trajectory.points)} points')
        
        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        
        # Validate joint names
        expected_joints = [
            f'{arm}_s0', f'{arm}_s1', f'{arm}_e0', f'{arm}_e1', 
            f'{arm}_w0', f'{arm}_w1', f'{arm}_w2'
        ]
        
        if not all(joint in expected_joints for joint in joint_names):
            self.get_logger().error(f'❌ Invalid joint names for {arm} arm: {joint_names}')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result
        
        # Execute each trajectory point
        start_time = self.get_clock().now()
        
        for i, point in enumerate(trajectory.points):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f'🛑 {arm.title()} arm trajectory canceled')
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                return result
            
            # Send command to real Baxter
            success = self.send_baxter_command(arm, joint_names, point.positions)
            
            if not success:
                self.get_logger().error(f'❌ Failed to send command for {arm} arm point {i+1}')
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result
            
            # Publish feedback
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = joint_names
            feedback.desired = point
            
            # Get actual positions for feedback
            actual_positions = []
            for joint_name in joint_names:
                actual_positions.append(self.current_joint_states.get(joint_name, 0.0))
            feedback.actual.positions = actual_positions
            feedback.actual.time_from_start = Duration(seconds=time.time() - start_time.nanoseconds/1e9).to_msg()
            
            goal_handle.publish_feedback(feedback)
            
            # Wait for point timing (simplified)
            if i < len(trajectory.points) - 1:
                # Calculate wait time based on trajectory timing
                current_time = point.time_from_start
                next_time = trajectory.points[i+1].time_from_start
                wait_duration = (next_time.sec + next_time.nanosec/1e9) - (current_time.sec + current_time.nanosec/1e9)
                if wait_duration > 0:
                    time.sleep(min(wait_duration, 5.0))  # Cap at 5 seconds per point
                else:
                    time.sleep(0.5)  # Minimum wait
            
            self.get_logger().info(f'✅ {arm.title()} arm point {i+1}/{len(trajectory.points)} executed')
        
        # Success!
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info(f'🎉 {arm.title()} arm trajectory completed successfully!')
        
        return result
    
    def send_baxter_command(self, arm, joint_names, positions):
        """Send joint position command to real Baxter"""
        try:
            # Create command script for Baxter
            joint_list = list(positions)
            
            script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('trajectory_executor', anonymous=True)
pub = rospy.Publisher('/robot/limb/{arm}/command_joint_position', JointState, queue_size=1)
rospy.sleep(1.0)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = {list(joint_names)}
msg.position = {joint_list}

print("Executing trajectory point for {arm} arm...")
print("Joints: {list(joint_names)}")
print("Positions: {joint_list}")

for i in range(10):
    pub.publish(msg)
    rospy.sleep(0.1)
    
print("Command sent to {arm} arm!")
'''
            
            # Write script to temp file
            with open(f'/tmp/baxter_{arm}_cmd.py', 'w') as f:
                f.write(script)
            
            # Execute via ROS 1 container
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', f'/tmp/baxter_{arm}_cmd.py:/tmp/baxter_{arm}_cmd.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                f'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/baxter_{arm}_cmd.py'
            ], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info(f'📡 Command sent to {arm} arm: {positions}')
                return True
            else:
                self.get_logger().error(f'❌ Command failed for {arm} arm: {result.stderr}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error sending command to {arm} arm: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    server = BaxterTrajectoryActionServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()