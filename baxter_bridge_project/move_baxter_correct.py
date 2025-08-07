#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time

def move_baxter_correctly():
    """Move Baxter using the correct topic"""
    
    rospy.init_node('move_baxter_correct', anonymous=True)
    
    # Create publishers for the correct topics
    enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
    left_pub = rospy.Publisher('/robot/limb/left/command_joint_position', JointState, queue_size=1)
    
    # Wait for publishers to connect
    rospy.sleep(3.0)
    
    print("Enabling Baxter robot...")
    
    # Enable the robot
    enable_msg = Bool()
    enable_msg.data = True
    for _ in range(5):
        enable_pub.publish(enable_msg)
        rospy.sleep(0.5)
    
    print("Robot enabled. Starting movement sequence...")
    
    # Define joint names for left arm
    joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    
    # Define movement sequence
    positions_sequence = [
        # Conservative start position
        [0.0, -0.3, 0.0, 0.5, 0.0, 0.8, 0.0],
        
        # Wave right
        [0.4, -0.3, 0.0, 0.5, 0.0, 0.8, 0.4],
        
        # Wave left  
        [-0.4, -0.3, 0.0, 0.5, 0.0, 0.8, -0.4],
        
        # Wave right again
        [0.4, -0.3, 0.0, 0.5, 0.0, 0.8, 0.4],
        
        # Back to start
        [0.0, -0.3, 0.0, 0.5, 0.0, 0.8, 0.0],
    ]
    
    # Execute movement sequence
    for i, positions in enumerate(positions_sequence):
        print("Moving to position {}/{}".format(i+1, len(positions_sequence)))
        
        # Create JointState message (correct format for Baxter)
        cmd = JointState()
        cmd.header.stamp = rospy.Time.now()
        cmd.name = joint_names
        cmd.position = positions
        
        # Publish command multiple times to ensure delivery
        for _ in range(10):
            left_pub.publish(cmd)
            rospy.sleep(0.1)
        
        # Wait between movements
        rospy.sleep(3.0)
    
    print("Movement sequence completed!")
    print("The robot should have moved its left arm in a waving motion.")

if __name__ == '__main__':
    try:
        move_baxter_correctly()
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except Exception as e:
        print("Error: {}".format(e))