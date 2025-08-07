#!/bin/bash

echo "🤖 Enabling Baxter with full environment setup..."

# Try with additional Baxter environment variables that might be needed
docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  -e ROS_IP=$(hostname -I | awk '{print $1}') \
  -e BAXTER_HOSTNAME=192.168.42.2 \
  baxter_ros1 \
  bash -c '
    source /opt/ros/indigo/setup.bash 
    source /ros/ws_baxter/devel/setup.bash
    
    # Set additional Baxter environment variables
    export ROS_IP=$(hostname -I | awk "{print \$1}")
    export BAXTER_HOSTNAME=192.168.42.2
    
    echo "Environment:"
    echo "ROS_MASTER_URI: $ROS_MASTER_URI"
    echo "ROS_IP: $ROS_IP" 
    echo "BAXTER_HOSTNAME: $BAXTER_HOSTNAME"
    echo ""
    
    echo "Testing rostopic list first..."
    if rostopic list | grep -q "/robot/state"; then
        echo "✅ Can see robot topics"
        echo ""
        echo "Trying enable command..."
        rosrun baxter_tools enable_robot.py -e
    else
        echo "❌ Cannot see robot topics"
    fi
  '

echo "✅ Enable attempt completed"