#!/usr/bin/env python

import rospy
from baxter_core_msgs.msg import JointCommand
import time
import math

def move_baxter_arm():
    """Move Baxter's left arm through a simple sequence"""
    
    rospy.init_node('move_baxter_real', anonymous=True)
    
    # Create publisher for left arm joint commands
    left_pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=1)
    
    # Wait for publisher to connect
    rospy.sleep(2.0)
    
    print("Starting Baxter arm movement test...")
    
    # Define joint names for left arm
    joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    
    # Define movement sequence (home -> wave positions)
    positions_sequence = [
        # Home position
        [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],
        
        # Wave right
        [0.5, -0.55, 0.0, 0.75, 0.0, 1.26, 0.5],
        
        # Wave left  
        [-0.5, -0.55, 0.0, 0.75, 0.0, 1.26, -0.5],
        
        # Wave right again
        [0.5, -0.55, 0.0, 0.75, 0.0, 1.26, 0.5],
        
        # Back to home
        [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0],
    ]
    
    # Execute movement sequence
    for i, positions in enumerate(positions_sequence):
        print("Moving to position {}/{}".format(i+1, len(positions_sequence)))
        
        # Create joint command message
        cmd = JointCommand()
        cmd.mode = JointCommand.POSITION_MODE
        cmd.names = joint_names
        cmd.command = positions
        
        # Publish command
        left_pub.publish(cmd)
        
        # Wait before next command
        time.sleep(3.0)
    
    print("Movement sequence completed!")

if __name__ == '__main__':
    try:
        move_baxter_arm()
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except Exception as e:
        print("Error: {}".format(e))