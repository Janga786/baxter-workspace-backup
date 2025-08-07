#!/bin/bash

echo "🤖 Testing Baxter Movement..."
echo "============================="

# Test robot movement
docker run --rm --network host \
  -v $(pwd):/scripts \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 bash -c \
  "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /scripts/move_baxter_correct.py"

echo ""
echo "✅ Movement test completed!"
echo "Did you see the robot's left arm move in a waving motion?"