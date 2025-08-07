#!/bin/bash

# Baxter Workspace Build Script using catkin_make
# This script builds only the core Baxter packages, excluding bridge projects

set -e  # Exit on error

echo "==========================================="
echo "Building Baxter Workspace with catkin_make"
echo "==========================================="

# Navigate to workspace
cd baxter_ws

# Source ROS environment
if [ -f "/opt/ros/indigo/setup.bash" ]; then
    source /opt/ros/indigo/setup.bash
    echo "✓ Sourced ROS Indigo environment"
elif [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo "✓ Sourced ROS Noetic environment"
else
    echo "⚠ No ROS environment found, please source manually"
fi

# Create CATKIN_IGNORE files to exclude bridge project folders
echo "Excluding bridge-related packages..."
IGNORE_DIRS=(
    "src/Other/baxter_bridge_project"
    "src/Other/moveit_ws" 
    "src/Other/sawyer_ws"
    "src/Other/ws_moveit"
    "src/Other/blender_output_detector"
)

for dir in "${IGNORE_DIRS[@]}"; do
    if [ -d "$dir" ]; then
        touch "$dir/CATKIN_IGNORE"
        echo "✓ Ignored: $dir"
    fi
done

# Clean previous build if it exists
if [ -d "build" ] || [ -d "devel" ]; then
    echo "Cleaning previous build..."
    rm -rf build devel
fi

# Build workspace with catkin_make
echo ""
echo "Building workspace..."
echo "Packages to build:"
echo "  - baxter_common (messages, description)"
echo "  - baxter_interface (robot interface)"  
echo "  - baxter_examples (demo scripts)"
echo "  - baxter_tools (utilities)"
echo "  - fort_lewis_demo (demo package)"
echo ""

catkin_make

# Check if build was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "✅ BUILD SUCCESSFUL!"
    echo "========================================="
    echo ""
    echo "To use the workspace, run:"
    echo "  source devel/setup.bash"
    echo ""
    echo "Core Baxter packages built:"
    echo "  - baxter_core_msgs: Message definitions"
    echo "  - baxter_description: Robot URDF and meshes"  
    echo "  - baxter_interface: Python API for robot control"
    echo "  - baxter_examples: Example scripts and demos"
    echo "  - baxter_tools: Calibration and maintenance tools"
    echo ""
    echo "Bridge projects excluded as requested."
else
    echo ""
    echo "❌ BUILD FAILED!"
    echo "Check the error messages above."
    exit 1
fi