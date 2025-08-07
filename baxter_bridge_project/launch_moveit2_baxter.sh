#!/bin/bash

echo "Starting MoveIt 2 for Baxter..."

# Terminal 1: Launch MoveIt Setup Assistant
gnome-terminal --title="MoveIt Setup Assistant" -- bash -c "
docker exec -it moveit_ros2_container bash -c '
source /opt/ros/humble/setup.bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
'; read -p 'Press enter to close...'"

echo ""
echo "MoveIt Setup Assistant launched!"
echo ""
echo "To create a MoveIt configuration for Baxter:"
echo "1. Click 'Create New MoveIt Configuration Package'"
echo "2. Load the URDF from: /ros/ws_moveit2/src/baxter_ros2/baxter_description/urdf/baxter.urdf"
echo "3. Follow the setup wizard to configure:"
echo "   - Self-Collisions"
echo "   - Virtual Joints (fixed base)"
echo "   - Planning Groups (left_arm, right_arm, both_arms)"
echo "   - Robot Poses"
echo "   - End Effectors"
echo "   - Controllers"
echo "4. Generate and save the package"

