#!/usr/bin/env python3

import subprocess
import time

def test_new_trajectory_system():
    """Test the new trajectory system"""
    
    print("🧪 Testing New Trajectory Action System")
    print("======================================")
    
    # Test sending a trajectory goal
    test_script = '''
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

rclpy.init()
node = Node("test_new_system")

# Subscribe to command topic to see if commands are being published
def cmd_callback(msg):
    print(f"📡 Command published: {msg.position}")

node.create_subscription(
    "sensor_msgs/JointState",
    "/baxter/left_arm/command", 
    cmd_callback,
    10
)

# Test action client
client = ActionClient(node, FollowJointTrajectory, "/left_arm_controller/follow_joint_trajectory")

print("Waiting for action server...")
if client.wait_for_server(timeout_sec=5.0):
    print("✅ Action server found!")
    
    # Create simple goal
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = ["left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"]
    
    point = JointTrajectoryPoint()
    point.positions = [0.1, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0]
    point.time_from_start = Duration(sec=2)
    goal.trajectory.points = [point]
    
    print("🎯 Sending test goal...")
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    goal_handle = future.result()
    if goal_handle and goal_handle.accepted:
        print("✅ Goal accepted!")
        
        # Wait for completion
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
        
        result = result_future.result()
        if result:
            print(f"🏁 Result: {result.result.error_code}")
        else:
            print("⏰ Timeout waiting for result")
    else:
        print("❌ Goal rejected")
else:
    print("❌ Action server not available")
    
rclpy.shutdown()
'''
    
    try:
        # Write test script
        with open('/tmp/test_new_system.py', 'w') as f:
            f.write(test_script)
        
        print("🚀 Running test inside container...")
        
        # Run test
        result = subprocess.run([
            'docker', 'exec', 'baxter_moveit', 'bash', '-c',
            'cd /shared_ws && source /opt/ros/humble/setup.bash && python3 /tmp/test_new_system.py'
        ], capture_output=True, text=True, timeout=20)
        
        print("📋 Test output:")
        print(result.stdout)
        if result.stderr:
            print("Errors:")
            print(result.stderr)
            
    except Exception as e:
        print(f"❌ Test error: {e}")

if __name__ == '__main__':
    test_new_trajectory_system()