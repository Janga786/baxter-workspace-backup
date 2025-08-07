#!/bin/bash

echo "🔍 Baxter System Diagnostics..."
echo "==============================="

# Run diagnostic script
docker run --rm --network host \
  -v $(pwd):/scripts \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 bash -c \
  "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /scripts/simple_baxter_check.py"

echo ""
echo "📖 See baxter_troubleshooting_guide.md for detailed solutions"