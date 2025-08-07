#!/bin/bash

echo "🤖 Starting Baxter ROS 2 MoveIt System..."
echo "========================================"

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

# Launch MoveIt with real robot integration
echo "🎯 Launching MoveIt with real Baxter integration..."
docker exec -d baxter_moveit bash -c \
  "cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch baxter_moveit_config baxter_real.launch.py"

# Wait for MoveIt to start
echo "⏳ Starting MoveIt services..."
sleep 8

# Check if nodes are running
echo "🔍 Checking system status..."
NODES=$(docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | wc -l)

if [ $NODES -gt 3 ]; then
    echo "✅ MoveIt system is running ($NODES nodes active)"
    echo ""
    echo "🎉 SUCCESS! Baxter MoveIt system is ready!"
    echo ""
    echo "📋 What's running:"
    echo "   - MoveIt motion planning"
    echo "   - RViz visualization"
    echo "   - Robot state publisher"
    echo "   - Transform broadcasting"
    echo ""
    echo "🎮 Next steps:"
    echo "   1. RViz should open automatically"
    echo "   2. Select planning group (left_arm/right_arm)"
    echo "   3. Drag orange robot to plan movements"
    echo "   4. Click 'Plan' then 'Execute'"
    echo ""
    echo "💻 To write your own code:"
    echo "   docker exec -it baxter_moveit bash"
    echo "   cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash"
    echo ""
    echo "🔧 To test robot movement:"
    echo "   ./test_baxter_movement.sh"
    echo ""
    echo "📖 See DAILY_QUICKSTART_GUIDE.md for complete reference"
else
    echo "❌ MoveIt startup may have failed"
    echo "🔍 Run diagnostics: ./diagnose_baxter.sh"
fi

echo ""
echo "🛑 To stop everything: docker stop baxter_moveit"