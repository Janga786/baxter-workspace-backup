#!/bin/bash

echo "🔍 Checking Baxter Status..."
echo "============================"

echo "1. Network connection to Baxter:"
if ping -c 2 192.168.42.2 > /dev/null 2>&1; then
    echo "✅ Can reach Baxter at 192.168.42.2"
else
    echo "❌ Cannot reach Baxter - check network"
    exit 1
fi

echo ""
echo "2. ROS Master on Baxter:"
if curl -s http://192.168.42.2:11311 > /dev/null 2>&1; then
    echo "✅ ROS Master responding"
else
    echo "❌ ROS Master not responding"
    exit 1
fi

echo ""
echo "3. Available robot topics:"
TOPICS=$(docker run --rm --network host -e ROS_MASTER_URI=http://192.168.42.2:11311 baxter_ros1 bash -c 'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rostopic list | grep "^/robot" | wc -l' 2>/dev/null)

if [ "$TOPICS" -gt 50 ]; then
    echo "✅ Found $TOPICS robot topics - Baxter ROS is running"
else
    echo "❌ Only found $TOPICS robot topics - Baxter may not be fully started"
fi

echo ""
echo "4. Testing robot state access:"
echo "Trying to get robot state (this may take a moment)..."

# Try to get robot state with timeout
STATE_CHECK=$(timeout 10 docker run --rm --network host -e ROS_MASTER_URI=http://192.168.42.2:11311 baxter_ros1 bash -c 'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && rostopic echo /robot/state -n 1' 2>&1)

if echo "$STATE_CHECK" | grep -q "enabled"; then
    echo "✅ Robot state accessible"
    if echo "$STATE_CHECK" | grep -q "enabled: True"; then
        echo "🎉 BAXTER IS ALREADY ENABLED!"
    else
        echo "⚠️ Baxter is not enabled yet"
    fi
else
    echo "❌ Cannot access robot state"
    echo "This usually means:"
    echo "  - Emergency stop is pressed"
    echo "  - Robot needs to be woken up (press cuff button)"
    echo "  - Baxter software is still starting"
fi

echo ""
echo "5. Recommended actions:"
echo "📋 Physical checks:"
echo "  - Make sure emergency stop button is OUT (not pressed)"
echo "  - Press one of the cuff buttons to wake up Baxter"
echo "  - Check that robot lights are on"
echo ""
echo "🔧 If problems persist:"
echo "  - Wait a few minutes for Baxter to fully boot"
echo "  - Try restarting Baxter's computer"
echo "  - Check Baxter's screen for error messages"