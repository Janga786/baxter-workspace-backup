#!/bin/bash

echo "🤖 Enabling Baxter (Direct Method)..."

# Send enable command directly via rostopic
echo "📡 Sending enable commands..."

for i in {1..5}; do
    echo "Enable attempt $i/5"
    docker run --rm --network host \
      -e ROS_MASTER_URI=http://192.168.42.2:11311 \
      baxter_ros1 \
      bash -c 'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rostopic pub /robot/set_super_enable std_msgs/Bool "data: true" -1' \
      > /dev/null 2>&1
    sleep 1
done

echo "✅ Enable commands sent!"
echo ""
echo "🔍 Checking robot state..."

# Check if robot is enabled
STATE=$(docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 \
  bash -c 'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && timeout 3 rostopic echo /robot/state -n 1 2>/dev/null | grep "enabled:"' 2>/dev/null)

if echo "$STATE" | grep -q "True"; then
    echo "🎉 SUCCESS! Baxter is enabled!"
    echo "✅ Robot is ready for movement commands"
else
    echo "⚠️  Enable status unclear - try checking:"
    echo "   1. Robot power and e-stop"
    echo "   2. Cuff button press to wake robot"
    echo "   3. Manual enable on robot if available"
fi

echo ""
echo "🧪 Test robot response:"
echo "docker exec baxter_moveit bash -c 'cd /shared_ws && source /opt/ros/humble/setup.bash && python3 simple_baxter_control.py'"