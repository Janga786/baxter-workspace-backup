#!/bin/bash

# Baxter Enable/Disable Script for ROS2 Integration

if [ "$1" = "--disable" ] || [ "$1" = "-d" ]; then
    echo "🔌 Disabling Baxter Robot..."
    
    cat > /tmp/disable_baxter.py << 'EOF'
import rospy
from std_msgs.msg import Bool

rospy.init_node('disable_baxter', anonymous=True)
enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
rospy.sleep(2.0)

disable_msg = Bool()
disable_msg.data = False

for i in range(5):
    enable_pub.publish(disable_msg)
    rospy.sleep(0.5)
    
print("Baxter disabled")
EOF

    docker run --rm --network host \
        -v /tmp/disable_baxter.py:/tmp/disable_baxter.py \
        -e ROS_MASTER_URI=http://192.168.42.2:11311 \
        baxter_ros1 bash -c \
        'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/disable_baxter.py'
        
    echo "✅ Baxter disabled"
    
else
    echo "🤖 Enabling Baxter Robot..."
    echo "=========================="
    
    cat > /tmp/enable_baxter.py << 'EOF'
import rospy
from std_msgs.msg import Bool

rospy.init_node('enable_baxter', anonymous=True)

print("Connecting to Baxter...")
enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
rospy.sleep(3.0)

print("Sending enable command...")
enable_msg = Bool()
enable_msg.data = True

for i in range(10):
    enable_pub.publish(enable_msg)
    print("Enable command " + str(i+1) + "/10 sent")
    rospy.sleep(0.5)
    
print("Baxter enable sequence completed!")
EOF

    echo "📡 Sending enable commands to Baxter..."
    
    docker run --rm --network host \
        -v /tmp/enable_baxter.py:/tmp/enable_baxter.py \
        -e ROS_MASTER_URI=http://192.168.42.2:11311 \
        baxter_ros1 bash -c \
        'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/enable_baxter.py'
    
    if [ $? -eq 0 ]; then
        echo ""
        echo "🎉 SUCCESS! Baxter should now be enabled!"
        echo "✅ Robot is ready for ROS2 movement commands"
        echo ""
        echo "🧪 Test with movement:"
        echo "docker exec baxter_moveit bash -c 'cd /shared_ws && source /opt/ros/humble/setup.bash && python3 simple_baxter_control.py'"
    else
        echo ""
        echo "❌ Enable command failed"
        echo "🔧 Check:"
        echo "  - Baxter is powered on"
        echo "  - Network connection to 192.168.42.2"
        echo "  - ROS1 master is running"
    fi
fi

echo ""
echo "📋 Usage:"
echo "  ./baxter_enable.sh          # Enable Baxter"
echo "  ./baxter_enable.sh --disable # Disable Baxter"