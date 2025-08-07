#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import time

def diagnose_baxter():
    """Diagnose Baxter's current state"""
    
    rospy.init_node('diagnose_baxter', anonymous=True)
    
    print("Diagnosing Baxter robot state...")
    
    # Check if we can connect to ROS master
    try:
        rospy.get_master().getSystemState()
        print("Connected to ROS master")
    except Exception as e:
        print("Failed to connect to ROS master: {}".format(e))
        return
    
    # List some key topics to check connectivity
    print("\nChecking key topics:")
    topics = rospy.get_published_topics()
    
    key_topics = [
        '/robot/limb/left/joint_command',
        '/robot/limb/right/joint_command', 
        '/robot/set_super_enable',
        '/robot/state'
    ]
    
    for topic_name in key_topics:
        found = any(topic[0] == topic_name for topic in topics)
        status = "OK" if found else "MISSING"
        print("  {} {}".format(status, topic_name))
    
    # Try to enable the robot multiple times
    print("\nAttempting to enable robot...")
    enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
    
    # Wait for publisher to connect
    rospy.sleep(2.0)
    
    for i in range(5):
        enable_msg = Bool()
        enable_msg.data = True
        enable_pub.publish(enable_msg)
        print("  Enable command {} sent".format(i+1))
        rospy.sleep(1.0)
    
    print("\nDiagnosis complete.")
    print("\nIf robot still doesn't move, please check:")
    print("1. Emergency stop button is not pressed (should be released)")
    print("2. Robot screen shows it's enabled")
    print("3. No error messages on robot display")
    print("4. Robot is powered on and calibrated")

if __name__ == '__main__':
    try:
        diagnose_baxter()
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except Exception as e:
        print("Error: {}".format(e))