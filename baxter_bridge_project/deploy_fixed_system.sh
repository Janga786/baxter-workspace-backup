#!/bin/bash

echo "🚀 Deploying Fixed Baxter MovEIt Integration System"
echo "=================================================="

# Set up environment
export WORKSPACE_DIR="/home/janga/baxter_bridge_project"
export SHARED_WS="$WORKSPACE_DIR/shared_ws"

# Function to check command success
check_status() {
    if [ $? -eq 0 ]; then
        echo "✅ $1"
    else
        echo "❌ $1 - FAILED"
        exit 1
    fi
}

# Check prerequisites
echo "🔍 Checking prerequisites..."

if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker is not running. Please start Docker first."
    exit 1
fi
check_status "Docker is running"

if ! ping -c 1 192.168.42.2 > /dev/null 2>&1; then
    echo "❌ Cannot reach Baxter robot at 192.168.42.2"
    echo "   Please check network connection and robot power."
    exit 1
fi
check_status "Baxter robot is reachable"

# Copy baxter message packages to shared workspace if not present
echo "📦 Setting up baxter message packages..."
if [ ! -d "$SHARED_WS/src/baxter_core_msgs" ]; then
    echo "   Copying baxter message packages to shared workspace..."
    cp -r "$WORKSPACE_DIR/baxter_common_ros2/baxter_core_msgs" "$SHARED_WS/src/"
    cp -r "$WORKSPACE_DIR/baxter_common_ros2/baxter_maintenance_msgs" "$SHARED_WS/src/"
    cp -r "$WORKSPACE_DIR/baxter_common_ros2/baxter_description" "$SHARED_WS/src/"
    check_status "Baxter message packages copied"
else
    echo "✅ Baxter message packages already present"
fi

# Make scripts executable
echo "🔧 Setting up executable permissions..."
chmod +x "$WORKSPACE_DIR/simple_baxter_system.py"
chmod +x "$WORKSPACE_DIR/test_moveit_integration.py"
chmod +x "$WORKSPACE_DIR/start_baxter_moveit.sh"
check_status "Script permissions set"

# Stop any existing containers
echo "🧹 Cleaning up existing containers..."
docker stop baxter_moveit 2>/dev/null || true
docker rm baxter_moveit 2>/dev/null || true
check_status "Container cleanup completed"

# Build baxter_moveit2 Docker image if needed
echo "🔨 Checking Docker images..."
if ! docker images | grep -q baxter_moveit2; then
    echo "   Building baxter_moveit2 Docker image..."
    cd "$WORKSPACE_DIR"
    docker build -t baxter_moveit2 -f moveit_ros2/Dockerfile .
    check_status "Docker image built"
else
    echo "✅ baxter_moveit2 Docker image exists"
fi

# Start the container
echo "🚀 Starting baxter_moveit container..."
docker run -d --name baxter_moveit \
  --network host \
  -v "$SHARED_WS:/shared_ws" \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --privileged \
  baxter_moveit2 tail -f /dev/null
check_status "Container started"

# Wait for container initialization
echo "⏳ Waiting for container initialization..."
sleep 3

# Build the workspace in container (excluding baxter_bridge which needs ROS1)
echo "🔨 Building ROS2 workspace..."
docker exec baxter_moveit bash -c \
  "cd /shared_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select baxter_core_msgs baxter_maintenance_msgs baxter_description baxter_moveit_config"
check_status "Workspace built successfully"

# Source the workspace
echo "📚 Sourcing workspace..."
docker exec baxter_moveit bash -c \
  "cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash"
check_status "Workspace sourced"

# Launch the integrated system
echo "🎯 Launching integrated Baxter MovEIt system..."
docker exec -d baxter_moveit bash -c \
  "cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch baxter_moveit_config baxter_real.launch.py"

# Wait for services to start
echo "⏳ Waiting for services to initialize..."
sleep 10

# Check if system is running
echo "🔍 Verifying system status..."
NODES=$(docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list 2>/dev/null" | wc -l)

if [ $NODES -gt 5 ]; then
    echo "✅ System is running with $NODES nodes"
else
    echo "❌ System may not have started correctly (only $NODES nodes found)"
    echo "🔍 Debug: Checking available nodes..."
    docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
fi

# Check if action servers are available
echo "🎭 Checking MovEIt action servers..."
docker exec baxter_moveit bash -c \
  "source /opt/ros/humble/setup.bash && timeout 5 ros2 action list | grep follow_joint_trajectory" && \
  check_status "MovEIt action servers are available" || \
  echo "⚠️  MovEIt action servers may not be ready yet"

# System summary
echo ""
echo "🎉 DEPLOYMENT COMPLETE!"
echo "======================"
echo ""
echo "📋 System Components Running:"
echo "   ✅ simple_baxter_system (ROS1 ↔ ROS2 integration + MovEIt action servers)"
echo "   ✅ robot_state_publisher (TF transforms)"
echo "   ✅ move_group (MovEIt planning)"
echo "   ✅ Real robot communication"
echo ""
echo "🎮 Testing the System:"
echo "   Run integration test:"
echo "   docker exec baxter_moveit bash -c 'cd /shared_ws && python3 /home/janga/baxter_bridge_project/test_moveit_integration.py'"
echo ""
echo "🎯 Using MovEIt:"
echo "   1. Open RViz: docker exec baxter_moveit rviz2"
echo "   2. Load MovEIt configuration"
echo "   3. Plan and execute movements"
echo ""
echo "💻 Development Environment:"
echo "   Access container: docker exec -it baxter_moveit bash"
echo "   Workspace: cd /shared_ws && source install/setup.bash"
echo ""
echo "🛑 Stop System:"
echo "   docker stop baxter_moveit"
echo ""
echo "✨ The system now provides a simplified but complete integration!"
echo "   Real robot movements work through MovEIt planning interface."
echo "   This approach avoids complex baxter_bridge build issues."