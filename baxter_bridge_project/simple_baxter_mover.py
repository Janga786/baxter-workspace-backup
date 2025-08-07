#!/usr/bin/env python3

import subprocess
import time

def move_baxter_simple(arm='left', positions=None):
    """Simple function to move Baxter directly"""
    
    if positions is None:
        positions = [0.1, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0]
    
    print(f"🤖 Moving {arm} arm to: {positions}")
    
    # Create ROS1 script
    script = f'''
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('simple_mover', anonymous=True)
pub = rospy.Publisher('/robot/limb/{arm}/command_joint_position', JointState, queue_size=1)
rospy.sleep(2.0)

msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = ['{arm}_s0', '{arm}_s1', '{arm}_e0', '{arm}_e1', '{arm}_w0', '{arm}_w1', '{arm}_w2']
msg.position = {positions}

print("Sending movement command...")
for i in range(15):
    pub.publish(msg)
    rospy.sleep(0.1)
    
print("Movement command sent!")
'''
    
    # Write and execute
    with open(f'/tmp/move_{arm}.py', 'w') as f:
        f.write(script)
    
    try:
        result = subprocess.run([
            'docker', 'run', '--rm', '--network', 'host',
            '-v', f'/tmp/move_{arm}.py:/tmp/move_{arm}.py',
            '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
            'baxter_ros1', 'bash', '-c',
            f'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/move_{arm}.py'
        ], capture_output=True, text=True, timeout=10)
        
        if result.returncode == 0:
            print(f"✅ {arm} arm command sent successfully!")
            print("👀 Baxter should be moving now!")
            return True
        else:
            print(f"❌ Failed: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def test_wave():
    """Test wave motion"""
    print("🌊 Testing wave motion...")
    
    positions = [
        [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],   # Center
        [0.3, -0.4, 0.0, 0.6, 0.0, 1.0, 0.3],   # Right
        [-0.3, -0.4, 0.0, 0.6, 0.0, 1.0, -0.3], # Left
        [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],   # Center
    ]
    
    for i, pos in enumerate(positions):
        print(f"Wave step {i+1}/4")
        if move_baxter_simple('left', pos):
            time.sleep(3)  # Wait between movements
        else:
            print("❌ Wave failed")
            return False
    
    print("🎉 Wave completed!")
    return True

if __name__ == '__main__':
    print("🤖 Simple Baxter Mover")
    print("=====================")
    print("This bypasses ROS2 action servers and sends commands directly")
    print("")
    
    choice = input("Test: (1) Single movement (2) Wave motion (q) Quit: ")
    
    if choice == '1':
        move_baxter_simple()
    elif choice == '2':
        test_wave()
    else:
        print("Exiting...")