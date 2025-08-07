#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import subprocess
import time
import threading


class BaxterMoveItBridge(Node):
    """
    Bridge that provides FollowJointTrajectory action servers for MovEIt
    while using the existing baxter_bridge for joint states and commands
    """
    
    def __init__(self):
        super().__init__('baxter_moveit_bridge')
        
        # Subscribe to joint states from baxter_bridge
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Action servers for MovEIt integration
        self.left_arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            self.execute_left_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.right_arm_server = ActionServer(
            self,
            FollowJointTrajectory, 
            '/right_arm_controller/follow_joint_trajectory',
            self.execute_right_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Current joint states for feedback
        self.current_joint_states = {}
        self.active_goals = {}
        
        self.get_logger().info('🚀 Baxter MovEIt Bridge started')
        self.get_logger().info('📡 Providing FollowJointTrajectory action servers:')
        self.get_logger().info('   - /left_arm_controller/follow_joint_trajectory')
        self.get_logger().info('   - /right_arm_controller/follow_joint_trajectory')
        
    def joint_state_callback(self, msg):
        """Store current joint states for feedback"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
    
    def goal_callback(self, goal_request):
        """Accept all goal requests"""
        self.get_logger().info('📩 Received trajectory goal')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Handle goal cancellation"""
        self.get_logger().info('🛑 Goal cancellation requested')
        return CancelResponse.ACCEPT
    
    async def execute_left_trajectory(self, goal_handle):
        """Execute trajectory on left arm"""
        return await self.execute_trajectory(goal_handle, 'left')
    
    async def execute_right_trajectory(self, goal_handle):
        """Execute trajectory on right arm"""
        return await self.execute_trajectory(goal_handle, 'right')
    
    async def execute_trajectory(self, goal_handle, arm):
        """Execute trajectory by commanding real Baxter robot"""
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points
        
        self.get_logger().info(f'🎯 Executing {arm} arm trajectory:')
        self.get_logger().info(f'   Joints: {joint_names}')
        self.get_logger().info(f'   Points: {len(points)}')
        
        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        
        # Validate joint names
        expected_joints = [
            f'{arm}_s0', f'{arm}_s1', f'{arm}_e0', f'{arm}_e1',
            f'{arm}_w0', f'{arm}_w1', f'{arm}_w2'
        ]
        
        if not all(joint in expected_joints for joint in joint_names):
            self.get_logger().error(f'❌ Invalid joint names for {arm} arm: {joint_names}')
            self.get_logger().error(f'   Expected: {expected_joints}')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return result
        
        # Store active goal
        goal_id = str(goal_handle.goal_id)
        self.active_goals[goal_id] = goal_handle
        
        try:
            # Execute each trajectory point
            start_time = self.get_clock().now()
            
            for i, point in enumerate(points):
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
                
                # Create and publish feedback
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = joint_names
                feedback.desired = point
                
                # Get actual positions for feedback
                actual_positions = []
                for joint_name in joint_names:
                    actual_positions.append(self.current_joint_states.get(joint_name, 0.0))
                
                feedback.actual.positions = actual_positions
                feedback.actual.velocities = [0.0] * len(joint_names)
                feedback.actual.accelerations = [0.0] * len(joint_names)
                
                # Calculate elapsed time
                elapsed_time = time.time() - start_time.nanoseconds / 1e9
                feedback.actual.time_from_start.sec = int(elapsed_time)
                feedback.actual.time_from_start.nanosec = int((elapsed_time % 1) * 1e9)
                
                goal_handle.publish_feedback(feedback)
                
                # Wait for timing between points
                if i < len(points) - 1:
                    current_time = point.time_from_start
                    next_time = points[i+1].time_from_start
                    
                    wait_time = (
                        (next_time.sec + next_time.nanosec / 1e9) -
                        (current_time.sec + current_time.nanosec / 1e9)
                    )
                    
                    if wait_time > 0:
                        time.sleep(min(wait_time, 3.0))  # Cap at 3 seconds
                    else:
                        time.sleep(0.2)  # Minimum wait
                
                self.get_logger().info(f'✅ {arm.title()} arm point {i+1}/{len(points)} completed')
            
            # Success!
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info(f'🎉 {arm.title()} arm trajectory executed successfully!')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error executing trajectory: {e}')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            
        finally:
            # Clean up
            if goal_id in self.active_goals:
                del self.active_goals[goal_id]
        
        return result
    
    def send_baxter_command(self, arm, joint_names, positions):
        """Send joint position command to real Baxter via ROS1"""
        try:
            # Create position list in correct order
            joint_positions = list(positions)
            
            # Create ROS1 command script
            script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('moveit_commander_{arm}', anonymous=True)
pub = rospy.Publisher('/robot/limb/{arm}/command_joint_position', JointState, queue_size=1)
rospy.sleep(0.5)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = {list(joint_names)}
msg.position = {joint_positions}

print(f"[MovEIt Bridge] Commanding {arm} arm:")
for name, pos in zip(msg.name, msg.position):
    print(f"  {{name}}: {{pos:.3f}}")

# Send command multiple times for reliability
for i in range(8):
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rospy.sleep(0.05)

print(f"[MovEIt Bridge] Command sent to {arm} arm successfully!")
'''
            
            # Write script to temp file
            script_file = f'/tmp/moveit_bridge_{arm}_{int(time.time()*1000)}.py'
            with open(script_file, 'w') as f:
                f.write(script)
            
            # Execute command via ROS1 container
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', f'{script_file}:{script_file}',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'timeout', '10',
                'bash', '-c',
                f'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python {script_file}'
            ], capture_output=True, text=True, timeout=15)
            
            # Clean up temp file
            try:
                import os
                os.remove(script_file)
            except:
                pass
            
            if result.returncode == 0:
                self.get_logger().info(f'📡 Command sent to {arm} arm: {[f"{p:.3f}" for p in positions]}')
                return True
            else:
                self.get_logger().error(f'❌ Command failed for {arm} arm. Return code: {result.returncode}')
                if result.stderr:
                    self.get_logger().error(f'   Error: {result.stderr.strip()}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Exception sending command to {arm} arm: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    bridge = BaxterMoveItBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('🛑 Shutting down Baxter MovEIt Bridge')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()