#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import subprocess
import time

def enable_baxter_direct():
    """Direct function to enable Baxter via ROS1"""
    print("🤖 Enabling Baxter Robot via ROS2...")
    print("====================================")
    
    try:
        # Simple enable script
        enable_script = '''
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
    print(f"Enable command {i+1}/10 sent")
    rospy.sleep(0.5)
    
print("✅ Baxter enable sequence completed!")
print("Robot should now be enabled and ready for movement.")
'''
        
        # Write script
        with open('/tmp/enable_baxter_simple.py', 'w') as f:
            f.write(enable_script)
        
        print("📡 Connecting to Baxter robot...")
        
        # Execute enable command
        result = subprocess.run([
            'docker', 'run', '--rm', '--network', 'host',
            '-v', '/tmp/enable_baxter_simple.py:/tmp/enable_baxter_simple.py',
            '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
            'baxter_ros1', 'bash', '-c',
            'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/enable_baxter_simple.py'
        ], capture_output=True, text=True, timeout=20)
        
        print("\n📋 Command Output:")
        print(result.stdout)
        
        if result.returncode == 0:
            print("\n🎉 SUCCESS!")
            print("✅ Baxter enable commands sent successfully")
            print("✅ Robot should now be enabled")
            print("\n🎯 Next steps:")
            print("1. Try moving Baxter with your ROS2 commands")
            print("2. Check if the robot lights are on")
            print("3. Test with: docker exec baxter_moveit bash -c 'cd /shared_ws && source /opt/ros/humble/setup.bash && python3 simple_baxter_control.py'")
        else:
            print("\n❌ FAILED!")
            print(f"Error: {result.stderr}")
            print("\n🔧 Troubleshooting:")
            print("1. Check Baxter is powered on")
            print("2. Verify network connection to 192.168.42.2")
            print("3. Make sure ROS1 master is running on Baxter")
            
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\n🔧 Try checking:")
        print("1. Baxter power and network connection")
        print("2. ROS1 master availability")

def disable_baxter_direct():
    """Direct function to disable Baxter via ROS1"""
    print("🔌 Disabling Baxter Robot...")
    
    try:
        disable_script = '''
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
'''
        
        with open('/tmp/disable_baxter_simple.py', 'w') as f:
            f.write(disable_script)
        
        result = subprocess.run([
            'docker', 'run', '--rm', '--network', 'host',
            '-v', '/tmp/disable_baxter_simple.py:/tmp/disable_baxter_simple.py',
            '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
            'baxter_ros1', 'bash', '-c',
            'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/disable_baxter_simple.py'
        ], timeout=10)
        
        if result.returncode == 0:
            print("✅ Baxter disabled successfully")
        else:
            print("❌ Failed to disable Baxter")
            
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == '__main__':
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == '--disable':
        disable_baxter_direct()
    else:
        enable_baxter_direct()