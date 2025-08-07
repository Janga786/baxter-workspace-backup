#!/usr/bin/env python3

import subprocess
import time

def test_direct_baxter_command():
    """Test sending a direct command to Baxter"""
    
    print("🧪 Testing direct command to Baxter...")
    
    # Test positions for left arm
    positions = [0.1, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0]
    
    try:
        # Create command script
        script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('direct_test', anonymous=True)
pub = rospy.Publisher('/robot/limb/left/command_joint_position', JointState, queue_size=1)
rospy.sleep(2.0)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
msg.position = {positions}

print("Sending command to left arm...")
for i in range(10):
    pub.publish(msg)
    rospy.sleep(0.1)
    
print("Command sent!")
'''
        
        # Write script
        with open('/tmp/test_direct_cmd.py', 'w') as f:
            f.write(script)
        
        print(f"📡 Sending test command: {positions}")
        
        # Execute command
        result = subprocess.run([
            'docker', 'run', '--rm', '--network', 'host',
            '-v', '/tmp/test_direct_cmd.py:/tmp/test_direct_cmd.py',
            '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
            'baxter_ros1', 'bash', '-c',
            'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/test_direct_cmd.py'
        ], capture_output=True, text=True, timeout=15)
        
        print(f"📋 Output: {result.stdout}")
        if result.stderr:
            print(f"❌ Errors: {result.stderr}")
            
        if result.returncode == 0:
            print("✅ Direct command sent successfully!")
            print("🤖 Check if Baxter moved!")
            return True
        else:
            print("❌ Command failed")
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == '__main__':
    test_direct_baxter_command()