#!/bin/bash

echo "🔌 Disabling Baxter using baxter_tools..."

docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 \
  bash -c 'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rosrun baxter_tools enable_robot.py -d'

echo "✅ Disable command completed"