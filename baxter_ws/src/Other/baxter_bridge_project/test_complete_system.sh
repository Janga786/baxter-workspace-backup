#!/bin/bash

echo "🧪 Testing Complete Baxter ROS 2 System"
echo "======================================"

# Check if container is running
if ! docker ps | grep -q baxter_moveit; then
    echo "❌ Baxter MoveIt container is not running"
    echo "   Start it first: ./start_complete_baxter_system.sh"
    exit 1
fi

echo "✅ Container is running"

# Test 1: Check node status
echo ""
echo "🔍 Test 1: Checking ROS 2 nodes..."
NODES=$(docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null")
echo "Active nodes:"
echo "$NODES"

# Test 2: Check action servers
echo ""
echo "🎯 Test 2: Checking action servers..."
ACTIONS=$(docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 action list 2>/dev/null")
echo "Available actions:"
echo "$ACTIONS"

# Test 3: Check if trajectory action servers are available
echo ""
echo "🤖 Test 3: Checking trajectory action servers..."
if echo "$ACTIONS" | grep -q "left_arm_controller/follow_joint_trajectory"; then
    echo "✅ Left arm trajectory action server: FOUND"
else
    echo "❌ Left arm trajectory action server: NOT FOUND"
fi

if echo "$ACTIONS" | grep -q "right_arm_controller/follow_joint_trajectory"; then
    echo "✅ Right arm trajectory action server: FOUND" 
else
    echo "❌ Right arm trajectory action server: NOT FOUND"
fi

# Test 4: Check joint state publishing
echo ""
echo "📊 Test 4: Checking joint states..."
JOINT_TEST=$(docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo /joint_states --once 2>/dev/null")
if [ $? -eq 0 ]; then
    echo "✅ Joint states are being published"
    echo "Sample joint state:"
    echo "$JOINT_TEST" | head -10
else
    echo "❌ Joint states not available"
fi

# Test 5: MoveIt planning test
echo ""
echo "🧠 Test 5: Testing MoveIt planning..."
docker exec baxter_moveit bash -c "
cd /shared_ws && 
source /opt/ros/humble/setup.bash && 
source install/setup.bash && 
python3 -c '
import rclpy
from rclpy.node import Node
import moveit_commander
import sys

rclpy.init()
node = Node(\"test_node\")

try:
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    left_arm = moveit_commander.MoveGroupCommander(\"left_arm\")
    
    print(\"✅ MoveIt interface working\")
    print(f\"Planning frame: {left_arm.get_planning_frame()}\")
    print(f\"End effector: {left_arm.get_end_effector_link()}\")
    print(f\"Joint names: {left_arm.get_joint_names()}\")
    
    # Test planning
    current = left_arm.get_current_joint_values()
    target = current.copy()
    target[0] += 0.1  # Small movement
    left_arm.set_joint_value_target(target)
    
    plan = left_arm.plan()
    if plan[0]:
        print(\"✅ Planning successful!\")
    else:
        print(\"❌ Planning failed\")
        
except Exception as e:
    print(f\"❌ MoveIt test failed: {e}\")
finally:
    rclpy.shutdown()
'" 2>/dev/null

# Test 6: Simple movement test
echo ""
echo "🎮 Test 6: Simple movement test..."
echo "This will send a small movement command to test the complete pipeline..."

read -p "Press Enter to test robot movement (this will move the left arm slightly)..."

docker exec baxter_moveit bash -c "
cd /shared_ws && 
source /opt/ros/humble/setup.bash && 
source install/setup.bash && 
timeout 15 python3 -c '
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

rclpy.init()
node = Node(\"movement_test\")

client = ActionClient(node, FollowJointTrajectory, \"/left_arm_controller/follow_joint_trajectory\")

print(\"Waiting for action server...\")
if client.wait_for_server(timeout_sec=5.0):
    print(\"✅ Action server found!\")
    
    # Create simple trajectory
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = JointTrajectory()
    goal.trajectory.joint_names = [\"left_s0\", \"left_s1\", \"left_e0\", \"left_e1\", \"left_w0\", \"left_w1\", \"left_w2\"]
    
    # Small movement
    point = JointTrajectoryPoint()
    point.positions = [0.1, -0.3, 0.0, 0.5, 0.0, 0.8, 0.0]
    point.time_from_start = Duration(sec=2)
    goal.trajectory.points = [point]
    
    print(\"Sending trajectory goal...\")
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    
    goal_handle = future.result()
    if goal_handle.accepted:
        print(\"✅ Goal accepted! Robot should be moving...\")
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
        
        result = result_future.result()
        if result and result.result.error_code == 0:
            print(\"🎉 Movement completed successfully!\")
        else:
            print(f\"❌ Movement failed with error: {result.result.error_code if result else 'timeout'}\")
    else:
        print(\"❌ Goal rejected\")
else:
    print(\"❌ Action server not available\")
    
rclpy.shutdown()
'" 2>/dev/null

echo ""
echo "📋 Test Summary:"
echo "==============="
echo "If all tests passed, your Baxter robot is fully integrated with ROS 2!"
echo ""
echo "You can now:"
echo "- Use RViz for interactive planning and execution"
echo "- Send ROS 2 trajectory goals that move the real robot"
echo "- Use MoveIt Python API for programmatic control"
echo "- Integrate with other ROS 2 packages seamlessly"
echo ""
echo "🎯 Try RViz: The robot should be visible and you can plan/execute movements"
echo "💻 For custom code: docker exec -it baxter_moveit bash"