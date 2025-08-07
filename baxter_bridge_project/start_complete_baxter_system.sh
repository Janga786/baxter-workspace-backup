#!/bin/bash

echo "🤖 Starting Complete Baxter ROS 2 MoveIt System..."
echo "==============================================="

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker first."
    exit 1
fi

# Check network connectivity to Baxter
echo "📡 Checking connection to Baxter..."
if ! ping -c 1 192.168.42.2 > /dev/null 2>&1; then
    echo "❌ Cannot reach Baxter at 192.168.42.2"
    echo "   Please check network connection and robot power."
    exit 1
fi
echo "✅ Baxter connection OK"

# Stop any existing containers
echo "🧹 Cleaning up existing containers..."
docker stop baxter_moveit 2>/dev/null || true
docker rm baxter_moveit 2>/dev/null || true

# Start MoveIt container
echo "🚀 Starting MoveIt container..."
docker run -d --name baxter_moveit \
  --network host \
  -v $(pwd)/shared_ws:/shared_ws \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --privileged \
  baxter_moveit2 tail -f /dev/null

if [ $? -eq 0 ]; then
    echo "✅ Container started successfully"
else
    echo "❌ Failed to start container"
    exit 1
fi

# Wait for container to be ready
echo "⏳ Waiting for container initialization..."
sleep 3

# Copy bridge files to container
echo "📁 Setting up bridge files..."
docker exec baxter_moveit chmod +x /shared_ws/real_robot_bridge.py
docker exec baxter_moveit chmod +x /shared_ws/baxter_trajectory_action_server.py

# Launch real robot configuration with bridges
echo "🎯 Launching complete Baxter system..."
docker exec -d baxter_moveit bash -c \
  "cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch baxter_moveit_config baxter_real.launch.py"

# Wait for system to start
echo "⏳ Starting complete system..."
sleep 10

# Check if nodes are running
echo "🔍 Checking system status..."
NODES=$(docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | wc -l)

if [ $NODES -gt 5 ]; then
    echo "✅ Complete Baxter system is running ($NODES nodes active)"
    echo ""
    echo "🎉 SUCCESS! Baxter is now fully controllable with ROS 2!"
    echo ""
    echo "📋 What's running:"
    echo "   - MoveIt motion planning ✅"
    echo "   - RViz visualization ✅"
    echo "   - Real robot bridge ✅"
    echo "   - Trajectory action servers ✅"
    echo "   - Joint state publishing ✅"
    echo ""
    echo "🎮 You can now:"
    echo "   1. Use RViz to plan and execute movements"
    echo "   2. Send ROS 2 commands that will move the real robot"
    echo "   3. Use MoveIt Python API for programmatic control"
    echo "   4. Execute trajectory goals via action clients"
    echo ""
    echo "🧪 Test with:"
    echo "   ./test_complete_system.sh"
    echo ""
    echo "💻 For development:"
    echo "   docker exec -it baxter_moveit bash"
    echo ""
else
    echo "❌ System startup may have failed"
    echo "🔍 Check logs: docker logs baxter_moveit"
fi

echo ""
echo "🛑 To stop everything: docker stop baxter_moveit"