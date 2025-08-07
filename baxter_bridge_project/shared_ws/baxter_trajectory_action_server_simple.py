#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time


class BaxterTrajectoryActionServerSimple(Node):
    def __init__(self):
        super().__init__('baxter_trajectory_action_server_simple')
        
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
        
        # Publishers to send commands to bridge
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
        
        self.get_logger().info('Baxter Trajectory Action Server (Simple) started')
        self.get_logger().info('Will publish commands to bridge topics')
        
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
        
        # Get the right publisher
        publisher = self.left_cmd_pub if arm == 'left' else self.right_cmd_pub
        
        # Execute each trajectory point
        start_time = self.get_clock().now()
        
        for i, point in enumerate(trajectory.points):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                self.get_logger().info(f'🛑 {arm.title()} arm trajectory canceled')
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                return result
            
            # Create and send command
            cmd_msg = JointState()
            cmd_msg.header = Header()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = list(joint_names)
            cmd_msg.position = list(point.positions)
            
            # Publish command to bridge
            publisher.publish(cmd_msg)
            
            self.get_logger().info(f'📡 Sent {arm} arm command: {point.positions}')
            
            # Publish feedback
            feedback.header.stamp = self.get_clock().now().to_msg()
            feedback.joint_names = joint_names
            feedback.desired = point
            
            # Get actual positions for feedback
            actual_positions = []
            for joint_name in joint_names:
                actual_positions.append(self.current_joint_states.get(joint_name, 0.0))
            feedback.actual.positions = actual_positions
            
            goal_handle.publish_feedback(feedback)
            
            # Wait for point timing
            if i < len(trajectory.points) - 1:
                current_time = point.time_from_start
                next_time = trajectory.points[i+1].time_from_start
                wait_duration = (next_time.sec + next_time.nanosec/1e9) - (current_time.sec + current_time.nanosec/1e9)
                if wait_duration > 0:
                    time.sleep(min(wait_duration, 3.0))
                else:
                    time.sleep(0.5)
            
            self.get_logger().info(f'✅ {arm.title()} arm point {i+1}/{len(trajectory.points)} executed')
        
        # Success!
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info(f'🎉 {arm.title()} arm trajectory completed!')
        
        return result


def main(args=None):
    rclpy.init(args=args)
    
    server = BaxterTrajectoryActionServerSimple()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()