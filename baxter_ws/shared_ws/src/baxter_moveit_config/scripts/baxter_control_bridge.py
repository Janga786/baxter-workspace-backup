#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.duration import Duration
import subprocess
import time
import threading


class BaxterControlBridge(Node):
    """
    Bridge that connects baxter_bridge joint states to ROS2 control system
    and provides FollowJointTrajectory action servers for MovEIt integration
    """
    
    def __init__(self):
        super().__init__('baxter_control_bridge')
        
        # Subscribe to joint states from baxter_bridge
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Republish joint states for ROS2 control
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Action servers for MoveIt integration
        self.left_arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/left_arm_controller/follow_joint_trajectory',
            self.execute_left_trajectory
        )
        
        self.right_arm_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/right_arm_controller/follow_joint_trajectory', 
            self.execute_right_trajectory
        )
        
        # Current joint states
        self.current_joint_states = {}
        
        self.get_logger().info('Baxter Control Bridge started')
        self.get_logger().info('Providing FollowJointTrajectory action servers for MovEIt')
        
    def joint_state_callback(self, msg):
        """Store current joint states and republish"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_states[name] = msg.position[i]
        
        # Republish for other nodes
        self.joint_state_pub.publish(msg)
    
    async def execute_left_trajectory(self, goal_handle):
        """Execute trajectory on left arm via baxter_bridge"""
        return await self.execute_trajectory(goal_handle, 'left')
    
    async def execute_right_trajectory(self, goal_handle):
        """Execute trajectory on right arm via baxter_bridge"""
        return await self.execute_trajectory(goal_handle, 'right')
    
    async def execute_trajectory(self, goal_handle, arm):
        """Execute trajectory by sending commands through baxter_bridge"""
        self.get_logger().info(f'🎯 Executing {arm} arm trajectory with {len(goal_handle.request.trajectory.points)} points')
        
        result = FollowJointTrajectory.Result()
        feedback = FollowJointTrajectory.Feedback()
        
        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        
        # Validate joint names for the arm
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
            
            # Send command to real Baxter through ROS1 bridge
            success = await self.send_baxter_command(arm, joint_names, point.positions)
            
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
            
            current_time = time.time() - start_time.nanoseconds / 1e9
            feedback.actual.time_from_start = Duration(seconds=current_time).to_msg()
            
            goal_handle.publish_feedback(feedback)
            
            # Wait for next point timing
            if i < len(trajectory.points) - 1:
                current_time_from_start = point.time_from_start
                next_time_from_start = trajectory.points[i+1].time_from_start
                
                wait_duration = (
                    (next_time_from_start.sec + next_time_from_start.nanosec / 1e9) -
                    (current_time_from_start.sec + current_time_from_start.nanosec / 1e9)
                )
                
                if wait_duration > 0:
                    await rclpy.spin_once(self, timeout_sec=min(wait_duration, 5.0))
                else:
                    await rclpy.spin_once(self, timeout_sec=0.5)
            
            self.get_logger().info(f'✅ {arm.title()} arm point {i+1}/{len(trajectory.points)} executed')
        
        # Success!
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info(f'🎉 {arm.title()} arm trajectory completed successfully!')
        
        return result
    
    async def send_baxter_command(self, arm, joint_names, positions):
        """Send joint position command to real Baxter via ROS1"""
        try:
            joint_list = list(positions)
            
            # Create command script for ROS1 Baxter
            script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('moveit_bridge_executor', anonymous=True)
pub = rospy.Publisher('/robot/limb/{arm}/command_joint_position', JointState, queue_size=1)
rospy.sleep(1.0)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = {list(joint_names)}
msg.position = {joint_list}

print("Executing MovEIt trajectory point for {arm} arm...")
print("Joints: {list(joint_names)}")
print("Positions: {joint_list}")

for i in range(15):
    pub.publish(msg)
    rospy.sleep(0.1)
    
print("Command sent to {arm} arm successfully!")
'''
            
            # Write script to temp file
            with open(f'/tmp/moveit_{arm}_cmd.py', 'w') as f:
                f.write(script)
            
            # Execute via ROS1 container
            result = subprocess.run([
                'docker', 'run', '--rm', '--network', 'host',
                '-v', f'/tmp/moveit_{arm}_cmd.py:/tmp/moveit_{arm}_cmd.py',
                '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
                'baxter_ros1', 'bash', '-c',
                f'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/moveit_{arm}_cmd.py'
            ], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0:
                self.get_logger().info(f'📡 MovEIt command sent to {arm} arm: {positions}')
                return True
            else:
                self.get_logger().error(f'❌ MovEIt command failed for {arm} arm: {result.stderr}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error sending MovEIt command to {arm} arm: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    bridge = BaxterControlBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()