#!/bin/bash

# Baxter Environment Setup Script
# This script sets up the environment for Baxter robot development

echo "==========================================="
echo "Setting up Baxter Development Environment"
echo "==========================================="

# Baxter robot IP - update this to match your robot
export BAXTER_HOSTNAME="baxter.local"  # or your robot's IP like "192.168.1.100"

# Set Baxter workspace
export BAXTER_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/baxter_ws" && pwd)"

# Source ROS environment
if [ -f "/opt/ros/indigo/setup.bash" ]; then
    source /opt/ros/indigo/setup.bash
    echo "✓ Sourced ROS Indigo"
elif [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    echo "✓ Sourced ROS Noetic"
else
    echo "⚠ Please install ROS (Indigo recommended for Baxter)"
    return 1
fi

# Source Baxter workspace if built
if [ -f "$BAXTER_WS/devel/setup.bash" ]; then
    source "$BAXTER_WS/devel/setup.bash"
    echo "✓ Sourced Baxter workspace"
else
    echo "⚠ Baxter workspace not built. Run ./build_baxter_workspace.sh first"
fi

# Source Baxter SDK environment setup
if [ -f "$BAXTER_WS/baxter.sh" ]; then
    # Set robot hostname for baxter.sh
    export BAXTER_HOSTNAME
    echo "✓ Baxter SDK environment ready"
    echo "  To enable robot, run: $BAXTER_WS/baxter.sh"
    echo "  Robot hostname set to: $BAXTER_HOSTNAME"
else
    echo "⚠ baxter.sh not found in workspace"
fi

echo ""
echo "Environment Setup Complete!"
echo "==========================================="
echo "Usage:"
echo "  1. Update BAXTER_HOSTNAME in this script to match your robot"
echo "  2. Run: source ./setup_baxter_environment.sh"  
echo "  3. Enable robot: cd baxter_ws && ./baxter.sh"
echo "  4. Run examples: rosrun baxter_examples joint_position_keyboard.py"
echo "==========================================="