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


class SimpleBaxterSystem(Node):
    """
    Simplified Baxter system that provides both joint state publishing 
    and MovEIt action servers without requiring the complex baxter_bridge
    """
    
    def __init__(self):
        super().__init__('simple_baxter_system')
        
        # Joint state publisher
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to publish joint states
        self.joint_state_timer = self.create_timer(0.1, self.publish_joint_states)
        
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
        
        # Current joint positions (will be updated from real robot)
        self.current_joint_positions = {
            'head_pan': 0.0,
            'left_s0': 0.0, 'left_s1': -0.55, 'left_e0': 0.0, 'left_e1': 0.75,
            'left_w0': 0.0, 'left_w1': 1.26, 'left_w2': 0.0,
            'right_s0': 0.0, 'right_s1': -0.55, 'right_e0': 0.0, 'right_e1': 0.75,
            'right_w0': 0.0, 'right_w1': 1.26, 'right_w2': 0.0
        }
        
        # Start thread to get real robot states
        self.joint_reader_thread = threading.Thread(target=self.read_robot_joints, daemon=True)
        self.joint_reader_thread.start()
        
        self.get_logger().info('🚀 Simple Baxter System started')
        self.get_logger().info('📡 Publishing joint states to /joint_states')
        self.get_logger().info('🎭 Providing MovEIt action servers:')
        self.get_logger().info('   - /left_arm_controller/follow_joint_trajectory')
        self.get_logger().info('   - /right_arm_controller/follow_joint_trajectory')
        
    def read_robot_joints(self):
        """Continuously read joint states from real robot"""
        while rclpy.ok():
            try:
                # Create script to get joint states from ROS1 Baxter
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
    rospy.signal_shutdown("Got joints")

rospy.init_node('joint_reader', anonymous=True)
rospy.Subscriber('/robot/joint_states', JointState, joint_callback)
rospy.spin()
'''
                
                # Write script to temp file
                with open('/tmp/read_joints.py', 'w') as f:
                    f.write(script)
                
                # Execute in ROS1 container to get joint states
                result = subprocess.run([
                    'docker', 'run', '--rm', '--network', 'host',
                    '-v', '/tmp/read_joints.py:/tmp/read_joints.py',
                    '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                    'baxter_ros1', 'timeout', '5',
                    'bash', '-c',
                    'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/read_joints.py'
                ], capture_output=True, text=True, timeout=8)
                
                if result.stdout.strip():
                    try:
                        lines = result.stdout.strip().split('\n')
                        for line in lines:
                            if line.startswith('{'):
                                joint_data = json.loads(line)
                                # Update current positions with real data
                                for joint_name, position in joint_data.items():
                                    if joint_name in self.current_joint_positions:
                                        self.current_joint_positions[joint_name] = position
                                break
                        
                        self.get_logger().info('📊 Updated joint states from real robot', once=True)
                    except json.JSONDecodeError as e:
                        self.get_logger().warn(f'JSON decode error: {e}', throttle_duration_sec=10.0)
                        
            except Exception as e:
                self.get_logger().warn(f'Error reading robot joints: {e}', throttle_duration_sec=10.0)
                
            time.sleep(0.5)  # Update at 2Hz
    
    def publish_joint_states(self):
        """Publish current joint states to ROS2"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''
        
        # Publish all joints
        msg.name = list(self.current_joint_positions.keys())
        msg.position = list(self.current_joint_positions.values())
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        
        self.joint_state_pub.publish(msg)
    
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
                    actual_positions.append(self.current_joint_positions.get(joint_name, 0.0))
                
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

rospy.init_node('simple_commander_{arm}', anonymous=True)
pub = rospy.Publisher('/robot/limb/{arm}/command_joint_position', JointState, queue_size=1)
rospy.sleep(0.5)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = {list(joint_names)}
msg.position = {joint_positions}

print(f"[Simple System] Commanding {arm} arm:")
for name, pos in zip(msg.name, msg.position):
    print(f"  {{name}}: {{pos:.3f}}")

# Send command multiple times for reliability
for i in range(8):
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rospy.sleep(0.05)

print(f"[Simple System] Command sent to {arm} arm successfully!")
'''
            
            # Write script to temp file
            script_file = f'/tmp/simple_cmd_{arm}_{int(time.time()*1000)}.py'
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
    
    system = SimpleBaxterSystem()
    
    try:
        rclpy.spin(system)
    except KeyboardInterrupt:
        system.get_logger().info('🛑 Shutting down Simple Baxter System')
    finally:
        system.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()