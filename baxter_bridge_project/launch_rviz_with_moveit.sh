#!/bin/bash

echo "🎮 Launching RViz with MoveIt for Baxter Control"
echo "=============================================="

# Kill any existing RViz
docker exec baxter_moveit pkill -f rviz2 2>/dev/null || true
sleep 2

echo "🚀 Starting RViz..."

# Launch RViz with MoveIt environment
docker exec baxter_moveit bash -c "
cd /shared_ws && 
source /opt/ros/humble/setup.bash && 
source install/setup.bash && 
export DISPLAY=$DISPLAY &&
export RVIZ2_PLUGINLIB_XML_PATHS=/opt/ros/humble/share/moveit_ros_visualization/plugin_description.xml &&
rviz2 -d /shared_ws/baxter_simple.rviz &
"

sleep 3

echo ""
echo "✅ RViz launched!"
echo ""
echo "🎯 To add Motion Planning:"
echo "1. In RViz, click 'Add' button"
echo "2. Look for 'moveit_rviz_plugin' tab"
echo "3. Select 'MotionPlanning' display"
echo "4. Set Planning Group to 'left_arm' or 'right_arm'"
echo ""
echo "🎮 Alternative - Use programmatic control:"
echo "docker exec -it baxter_moveit bash"
echo "cd /shared_ws && source /opt/ros/humble/setup.bash && python3 simple_baxter_control.py"
echo ""
echo "🛑 To stop RViz: docker exec baxter_moveit pkill -f rviz2"