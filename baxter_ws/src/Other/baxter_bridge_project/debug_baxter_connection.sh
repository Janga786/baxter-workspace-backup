#!/bin/bash

echo "🔍 Baxter Connection Debug"
echo "========================="

echo ""
echo "1. Testing basic network connectivity..."
if ping -c 2 192.168.42.2 > /dev/null 2>&1; then
    echo "✅ Network ping to Baxter successful"
else
    echo "❌ Cannot ping Baxter robot at 192.168.42.2"
    echo "   Please check network connection"
    exit 1
fi

echo ""
echo "2. Testing ROS Master connection..."
export ROS_MASTER_URI=http://192.168.42.2:11311
if docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 timeout 5 \
  bash -c "source /opt/ros/indigo/setup.bash && rostopic list > /dev/null 2>&1"; then
    echo "✅ Can connect to ROS Master"
else
    echo "❌ Cannot connect to ROS Master"
    echo "   Check if Baxter's ROS system is running"
    exit 1
fi

echo ""
echo "3. Checking available ROS topics..."
echo "First 10 topics:"
docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 \
  bash -c "source /opt/ros/indigo/setup.bash && rostopic list | head -10"

echo ""
echo "4. Checking robot state topic..."
if docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 \
  bash -c "source /opt/ros/indigo/setup.bash && rostopic list | grep '/robot/state'"; then
    echo "✅ Robot state topic exists"
    
    echo ""
    echo "5. Testing robot state message..."
    echo "Trying to read robot state (timeout 3 seconds)..."
    
    STATE_MSG=$(docker run --rm --network host \
      -e ROS_MASTER_URI=http://192.168.42.2:11311 \
      baxter_ros1 timeout 3 \
      bash -c "source /opt/ros/indigo/setup.bash && rostopic echo /robot/state -n 1 2>/dev/null" 2>/dev/null)
    
    if [ ! -z "$STATE_MSG" ]; then
        echo "✅ Can read robot state"
        echo "Robot state info:"
        echo "$STATE_MSG" | grep -E "(enabled|stopped|error)"
    else
        echo "⚠️  Robot state topic exists but no data received"
        echo "   This might mean the robot is not fully booted"
    fi
else
    echo "❌ Robot state topic not found"
    echo "   Robot may not be fully started"
fi

echo ""
echo "6. Checking joint states topic..."
if docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 \
  bash -c "source /opt/ros/indigo/setup.bash && rostopic list | grep '/robot/joint_states'"; then
    echo "✅ Joint states topic exists"
    
    echo "Testing joint states data..."
    JOINT_DATA=$(docker run --rm --network host \
      -e ROS_MASTER_URI=http://192.168.42.2:11311 \
      baxter_ros1 timeout 2 \
      bash -c "source /opt/ros/indigo/setup.bash && rostopic echo /robot/joint_states -n 1 2>/dev/null" 2>/dev/null)
    
    if [ ! -z "$JOINT_DATA" ]; then
        echo "✅ Joint states are being published"
        JOINT_COUNT=$(echo "$JOINT_DATA" | grep -c "name:")
        echo "   Found joint data with $JOINT_COUNT joint names"
    else
        echo "⚠️  Joint states topic exists but no data"
    fi
else
    echo "❌ Joint states topic not found"
fi

echo ""
echo "7. Summary and Recommendations:"
echo "==============================="

# Check if we can get basic robot info
TOPICS_COUNT=$(docker run --rm --network host \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 \
  bash -c "source /opt/ros/indigo/setup.bash && rostopic list 2>/dev/null | wc -l" 2>/dev/null)

echo "📊 Total ROS topics available: $TOPICS_COUNT"

if [ "$TOPICS_COUNT" -gt 50 ]; then
    echo "✅ Robot appears to be running normally"
    echo ""
    echo "🎯 Next steps:"
    echo "1. Try manual enable with the enable button on robot"
    echo "2. Test our system without full enable:"
    echo "   docker exec baxter_moveit bash -c 'cd /shared_ws && python3 /tmp/simple_baxter_system.py &'"
    echo "3. Check if joint states flow to ROS2:"
    echo "   docker exec baxter_moveit bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /joint_states'"
elif [ "$TOPICS_COUNT" -gt 10 ]; then
    echo "⚠️  Robot is partially running"
    echo ""
    echo "🔧 Possible solutions:"
    echo "1. Restart robot software"
    echo "2. Check robot power and startup sequence"
    echo "3. Verify network configuration"
else
    echo "❌ Robot does not appear to be running properly"
    echo ""
    echo "🚨 Required actions:"
    echo "1. Power cycle the robot"
    echo "2. Check all cable connections"
    echo "3. Verify robot is on the same network"
fi

echo ""
echo "🧪 Test our system anyway:"
echo "Even if enable fails, our bridge should still work with read-only data"
echo "Run: ./test_readonly_system.sh"