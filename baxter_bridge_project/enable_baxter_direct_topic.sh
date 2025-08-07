#!/bin/bash

echo "🤖 Enabling Baxter via direct topic (bypassing baxter_interface)..."

# Send enable directly to the topic that baxter_tools would use
docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  -e ROS_IP=$(hostname -I | awk '{print $1}') \
  baxter_ros1 \
  bash -c '
    source /opt/ros/indigo/setup.bash 
    source /ros/ws_baxter/devel/setup.bash
    
    echo "Sending enable command directly to /robot/set_super_enable..."
    
    # Send enable command multiple times to ensure it gets through
    for i in {1..5}; do
        echo "Enable attempt $i/5"
        rostopic pub /robot/set_super_enable std_msgs/Bool "data: true" -1
        sleep 1
    done
    
    echo "Enable commands sent!"
  '

echo "✅ Direct enable completed"

# Test if it worked by trying to get a simple topic
echo "🔍 Testing if robot is responsive..."
docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 \
  bash -c '
    source /opt/ros/indigo/setup.bash 
    source /ros/ws_baxter/devel/setup.bash
    echo "Checking joint states..."
    timeout 3 rostopic echo /robot/joint_states -n 1 | head -5
  ' 2>/dev/null

if [ $? -eq 0 ]; then
    echo "✅ Robot seems to be responding!"
else
    echo "⚠️ Robot may need more time or additional setup"
fi