#!/bin/bash

echo "🎮 Launching RViz for Baxter Control"
echo "=================================="

# Make sure X11 forwarding works
xhost +local:docker 2>/dev/null

# Kill any existing RViz processes
docker exec baxter_moveit pkill -f rviz2 2>/dev/null || true

# Wait a moment
sleep 2

echo "🚀 Starting RViz with Baxter configuration..."

# Launch RViz with MoveIt motion planning plugin
docker exec baxter_moveit bash -c "
cd /shared_ws && 
source /opt/ros/humble/setup.bash && 
source install/setup.bash && 
export DISPLAY=$DISPLAY &&
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/moveit_resources_panda_moveit_config/config/moveit.rviz &
"

# Wait for RViz to start
sleep 3

echo ""
echo "🎯 RViz Instructions:"
echo "==================="
echo "1. In RViz, add 'MotionPlanning' display if not present"
echo "2. Set Fixed Frame to 'base' or 'world'"
echo "3. In Motion Planning panel:"
echo "   - Planning Group: Select 'left_arm' or 'right_arm'"
echo "   - Enable 'Show Goal State' and 'Show Trail'"
echo "4. Drag the orange interactive marker to plan movements"
echo "5. Click 'Plan' to generate trajectory"
echo "6. Click 'Execute' to move the real Baxter!"
echo ""
echo "✅ Your ROS 2 commands will now control the physical robot!"
echo ""
echo "🛑 To stop: docker exec baxter_moveit pkill -f rviz2"