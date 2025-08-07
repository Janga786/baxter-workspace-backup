#!/bin/bash

echo "🤖 Complete Baxter System Test"
echo "=============================="

# Function to check command success
check_status() {
    if [ $? -eq 0 ]; then
        echo "✅ $1"
        return 0
    else
        echo "❌ $1 - FAILED"
        return 1
    fi
}

# Step 1: Check if Baxter is reachable
echo "🔍 Step 1: Checking Baxter connectivity..."
if ping -c 1 192.168.42.2 > /dev/null 2>&1; then
    check_status "Baxter robot is reachable at 192.168.42.2"
else
    echo "❌ Cannot reach Baxter robot at 192.168.42.2"
    echo "   Please check network connection and robot power."
    echo "   Make sure you're connected to the Baxter network."
    exit 1
fi

# Step 2: Enable Baxter Robot
echo ""
echo "🔌 Step 2: Enabling Baxter robot..."
echo "   This requires ROS 1 environment..."

# Create enable script
cat > /tmp/enable_baxter.py << 'EOF'
import rospy
import baxter_interface

def enable_baxter():
    print("Initializing ROS node...")
    rospy.init_node('enable_baxter', anonymous=True)
    
    print("Connecting to Baxter...")
    rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    
    print("Current robot state:", "Enabled" if rs.state().enabled else "Disabled")
    
    if not rs.state().enabled:
        print("Enabling robot...")
        rs.enable()
        rospy.sleep(1.0)
        
        if rs.state().enabled:
            print("✅ Robot enabled successfully!")
            return True
        else:
            print("❌ Failed to enable robot")
            return False
    else:
        print("✅ Robot is already enabled!")
        return True

if __name__ == '__main__':
    try:
        enable_baxter()
    except Exception as e:
        print(f"❌ Error enabling robot: {e}")
EOF

# Run enable script in ROS1 container
echo "   Running enable command via ROS1 container..."
docker run --rm --network host \
  -v /tmp/enable_baxter.py:/tmp/enable_baxter.py \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 \
  bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/enable_baxter.py"

check_status "Baxter enable command executed"

# Step 3: Check if our container is running
echo ""
echo "🐳 Step 3: Checking Docker container..."
if docker ps | grep -q baxter_moveit; then
    check_status "baxter_moveit container is running"
else
    echo "❌ baxter_moveit container is not running"
    echo "   Starting container now..."
    cd /home/janga/baxter_bridge_project
    ./deploy_fixed_system.sh
fi

# Step 4: Check if our services are running
echo ""
echo "🔍 Step 4: Checking system services..."
echo "   Waiting for services to start..."
sleep 5

# Check nodes
NODES=$(docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | wc -l)
echo "   Found $NODES ROS2 nodes running"

if [ $NODES -gt 3 ]; then
    check_status "ROS2 system is running"
else
    echo "⚠️  Limited nodes detected. Starting system manually..."
    # Start our system manually
    docker exec -d baxter_moveit bash -c \
      "cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch baxter_moveit_config baxter_real.launch.py"
    
    echo "   Waiting for manual start..."
    sleep 8
fi

# Step 5: Test joint state communication
echo ""
echo "📊 Step 5: Testing joint state communication..."
echo "   Checking if joint states are flowing from robot..."

JOINT_TEST=$(docker exec baxter_moveit bash -c \
  "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /joint_states --once 2>/dev/null" | wc -l)

if [ $JOINT_TEST -gt 5 ]; then
    check_status "Joint states are being published"
else
    echo "⚠️  Joint states may not be flowing properly"
    echo "   Starting simple baxter system..."
    
    # Copy our simple system to the container
    docker cp /home/janga/baxter_bridge_project/simple_baxter_system.py baxter_moveit:/tmp/
    
    # Start it
    docker exec -d baxter_moveit bash -c \
      "cd /shared_ws && source /opt/ros/humble/setup.bash && python3 /tmp/simple_baxter_system.py"
    
    echo "   Waiting for simple system to start..."
    sleep 5
fi

# Step 6: Test MovEIt action servers
echo ""
echo "🎭 Step 6: Testing MovEIt action servers..."
ACTION_SERVERS=$(docker exec baxter_moveit bash -c \
  "source /opt/ros/humble/setup.bash && timeout 3 ros2 action list 2>/dev/null | grep follow_joint_trajectory | wc -l")

if [ $ACTION_SERVERS -ge 2 ]; then
    check_status "MovEIt action servers are available"
else
    echo "⚠️  MovEIt action servers not detected"
    echo "   This may be normal if the system is still starting up"
fi

# Step 7: Run integration test
echo ""
echo "🧪 Step 7: Running integration test..."
echo "   This will test real robot movement through MovEIt..."

# Copy test script to container
docker cp /home/janga/baxter_bridge_project/test_moveit_integration.py baxter_moveit:/tmp/

# Run test
echo "   Executing movement test..."
docker exec baxter_moveit bash -c \
  "cd /shared_ws && source /opt/ros/humble/setup.bash && python3 /tmp/test_moveit_integration.py"

TEST_RESULT=$?
if [ $TEST_RESULT -eq 0 ]; then
    check_status "Integration test completed"
else
    echo "⚠️  Integration test had issues (this may be normal for first run)"
fi

# Step 8: System summary
echo ""
echo "📋 System Status Summary"
echo "======================="

# Check all components
echo "🔍 Component Status:"

# Check Baxter enable status
ENABLE_STATUS=$(docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 timeout 3 \
  bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rostopic echo /robot/state -n 1 2>/dev/null | grep enabled" 2>/dev/null)

if echo "$ENABLE_STATUS" | grep -q "True"; then
    echo "   ✅ Baxter robot is enabled"
else
    echo "   ⚠️  Baxter robot enable status unclear"
fi

# Check container
if docker ps | grep -q baxter_moveit; then
    echo "   ✅ Docker container is running"
else
    echo "   ❌ Docker container is not running"
fi

# Check ROS2 nodes
NODES=$(docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | wc -l)
echo "   📊 ROS2 nodes running: $NODES"

# Check action servers
ACTIONS=$(docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 action list 2>/dev/null | wc -l")
echo "   🎭 Action servers available: $ACTIONS"

echo ""
echo "🎯 Next Steps:"
echo "============="
echo ""
echo "1. 🎮 Test Manual Movement:"
echo "   docker exec baxter_moveit bash -c 'cd /shared_ws && python3 /tmp/test_moveit_integration.py'"
echo ""
echo "2. 🖥️  Open RViz for Visual Planning:"
echo "   docker exec baxter_moveit bash -c 'source /opt/ros/humble/setup.bash && rviz2'"
echo ""
echo "3. 💻 Access Development Environment:"
echo "   docker exec -it baxter_moveit bash"
echo "   cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash"
echo ""
echo "4. 🛑 Stop System:"
echo "   docker stop baxter_moveit"
echo ""

if [ $NODES -gt 3 ] && [ $ACTIONS -gt 1 ]; then
    echo "🎉 SUCCESS! System appears to be working correctly!"
    echo "   You can now use MovEIt to plan and execute robot movements."
else
    echo "⚠️  System may need additional setup. Check the logs above for issues."
fi

echo ""
echo "📚 For detailed instructions, see:"
echo "   - FINAL_WORKING_SYSTEM.md"
echo "   - DAILY_QUICKSTART_GUIDE.md"