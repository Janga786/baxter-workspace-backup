#!/bin/bash

echo "🤖 Enabling Baxter using your working method..."

# Add baxter.local to hosts in our container
docker exec baxter_moveit bash -c 'echo "192.168.42.2   baxter.local baxter" >> /etc/hosts'

# Try the enable command with the hosts entry
docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 \
  bash -c '
    echo "192.168.42.2   baxter.local baxter" >> /etc/hosts
    source /opt/ros/indigo/setup.bash 
    source /ros/ws_baxter/devel/setup.bash
    
    echo "Trying enable with baxter.local hostname..."
    rosrun baxter_tools enable_robot.py -e
  '

echo "✅ Enable attempt with hostname completed"