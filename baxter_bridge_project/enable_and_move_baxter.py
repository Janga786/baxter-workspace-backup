#!/usr/bin/env python

import rospy
from baxter_core_msgs.msg import JointCommand
from std_msgs.msg import Empty, Bool
import time

def enable_and_move_baxter():
    """Enable Baxter and move its left arm"""
    
    rospy.init_node('enable_and_move_baxter', anonymous=True)
    
    # Create publishers
    enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
    left_pub = rospy.Publisher('/robot/limb/left/joint_command', JointCommand, queue_size=1)
    
    # Wait for publishers to connect
    rospy.sleep(2.0)
    
    print("Enabling Baxter robot...")
    
    # Enable the robot
    enable_msg = Bool()
    enable_msg.data = True
    enable_pub.publish(enable_msg)
    
    # Wait for robot to enable
    rospy.sleep(3.0)
    
    print("Robot enabled. Starting movement sequence...")
    
    # Define joint names for left arm
    joint_names = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
    
    # Define movement sequence (more conservative movements)
    positions_sequence = [
        # Start from current position (neutral)
        [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],
        
        # Small wave right
        [0.3, -0.4, 0.0, 0.6, 0.0, 1.0, 0.3],
        
        # Small wave left  
        [-0.3, -0.4, 0.0, 0.6, 0.0, 1.0, -0.3],
        
        # Wave right again
        [0.3, -0.4, 0.0, 0.6, 0.0, 1.0, 0.3],
        
        # Back to neutral
        [0.0, -0.4, 0.0, 0.6, 0.0, 1.0, 0.0],
    ]
    
    # Execute movement sequence
    for i, positions in enumerate(positions_sequence):
        print("Moving to position {}/{}".format(i+1, len(positions_sequence)))
        
        # Create joint command message
        cmd = JointCommand()
        cmd.mode = JointCommand.POSITION_MODE
        cmd.names = joint_names
        cmd.command = positions
        
        # Publish command multiple times to ensure it's received
        for _ in range(5):
            left_pub.publish(cmd)
            rospy.sleep(0.1)
        
        # Wait longer between movements for safety
        rospy.sleep(4.0)
    
    print("Movement sequence completed!")
    
    # Optionally disable robot for safety
    print("Disabling robot for safety...")
    enable_msg.data = False
    enable_pub.publish(enable_msg)

if __name__ == '__main__':
    try:
        enable_and_move_baxter()
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except Exception as e:
        print("Error: {}".format(e))