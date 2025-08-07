#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time
import threading
import subprocess
import tempfile
import os


class BaxterTrajectoryActionServerFixed(Node):
    def __init__(self):
        super().__init__('baxter_trajectory_action_server_fixed')
        
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
        
        # Publisher to send commands directly via ROS 2
        self.left_cmd_pub = self.create_publisher(
            JointState, 
            '/baxter/left_arm/command', 
            10
        )
        
        self.right_cmd_pub = self.create_publisher(
            JointState, 
            '/baxter/right_arm/command', 
            10
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
        
        # Timer to send commands to real robot
        self.command_timer = self.create_timer(0.1, self.send_pending_commands)
        self.pending_commands = []
        
        self.get_logger().info('Baxter Trajectory Action Server (Fixed) started')
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
            success = self.send_baxter_command_direct(arm, joint_names, point.positions)
            
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
    
    def send_baxter_command_direct(self, arm, joint_names, positions):
        """Send joint position command to real Baxter via external script"""
        try:
            # Add command to pending queue for external execution
            command = {
                'arm': arm,
                'joint_names': list(joint_names),
                'positions': list(positions),
                'timestamp': time.time()
            }
            self.pending_commands.append(command)
            
            self.get_logger().info(f'📡 Queued command for {arm} arm: {positions}')
            return True
                
        except Exception as e:
            self.get_logger().error(f'❌ Error queuing command for {arm} arm: {e}')
            return False
    
    def send_pending_commands(self):
        """Send any pending commands to real robot"""
        if not self.pending_commands:
            return
            
        # Take the most recent command for each arm
        latest_commands = {}
        for cmd in self.pending_commands:
            latest_commands[cmd['arm']] = cmd
        
        # Clear pending commands
        self.pending_commands.clear()
        
        # Send commands
        for arm, cmd in latest_commands.items():
            self.execute_baxter_command_external(cmd)
    
    def execute_baxter_command_external(self, command):
        """Execute command on real Baxter using external script"""
        try:
            arm = command['arm']
            positions = command['positions']
            
            # Create temp script
            script_content = f'''#!/bin/bash
# Auto-generated Baxter command script
docker run --rm --network host \\
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \\
  baxter_ros1 \\
  bash -c '
    source /opt/ros/indigo/setup.bash && 
    source /ros/ws_baxter/devel/setup.bash && 
    rostopic pub /robot/limb/{arm}/command_joint_position sensor_msgs/JointState "{{
      header: {{stamp: now}},
      name: [\\"{arm}_s0\\", \\"{arm}_s1\\", \\"{arm}_e0\\", \\"{arm}_e1\\", \\"{arm}_w0\\", \\"{arm}_w1\\", \\"{arm}_w2\\"],
      position: {positions}
    }}" -1
  '
'''
            
            # Write script to shared location
            script_path = f'/shared_ws/baxter_cmd_{arm}.sh'
            with open(script_path, 'w') as f:
                f.write(script_content)
            os.chmod(script_path, 0o755)
            
            # Execute via subprocess (run outside container)
            result = subprocess.run([
                'bash', '-c', f'cd /home/janga/baxter_bridge_project && {script_path}'
            ], capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self.get_logger().info(f'✅ Command sent to {arm} arm successfully')
            else:
                self.get_logger().warn(f'⚠️ Command to {arm} arm may have failed: {result.stderr}')
                
        except Exception as e:
            self.get_logger().error(f'❌ Error executing command: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    server = BaxterTrajectoryActionServerFixed()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()