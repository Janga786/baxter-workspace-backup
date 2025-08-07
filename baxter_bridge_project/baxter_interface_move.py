#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import time

def move_baxter_with_interface():
    """Use baxter_interface to move the robot properly"""
    
    print("Initializing node...")
    rospy.init_node("baxter_interface_move")
    
    print("Getting robot enable state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    
    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    
    print("Enabling robot...")
    rs.enable()
    
    print("Getting left limb interface...")
    left = baxter_interface.Limb('left')
    
    # Set joint position speed
    left.set_joint_position_speed(0.3)
    
    print("Moving to start position...")
    # Move to a safe start position
    start_angles = {
        'left_w0': 0.0,
        'left_w1': 1.0,
        'left_w2': 0.0,
        'left_e0': 0.0,
        'left_e1': 0.5,
        'left_s0': 0.0,
        'left_s1': -0.5
    }
    
    left.move_to_joint_positions(start_angles)
    rospy.sleep(2.0)
    
    print("Starting wave motion...")
    # Define wave sequence
    wave_positions = [
        # Wave right
        {
            'left_w0': 0.0,
            'left_w1': 1.0,
            'left_w2': 0.5,
            'left_e0': 0.0,
            'left_e1': 0.5,
            'left_s0': 0.5,
            'left_s1': -0.5
        },
        # Wave left
        {
            'left_w0': 0.0,
            'left_w1': 1.0,
            'left_w2': -0.5,
            'left_e0': 0.0,
            'left_e1': 0.5,
            'left_s0': -0.5,
            'left_s1': -0.5
        },
        # Wave right again
        {
            'left_w0': 0.0,
            'left_w1': 1.0,
            'left_w2': 0.5,
            'left_e0': 0.0,
            'left_e1': 0.5,
            'left_s0': 0.5,
            'left_s1': -0.5
        }
    ]
    
    # Execute wave motions
    for i, position in enumerate(wave_positions):
        print("Wave motion {}/{}".format(i+1, len(wave_positions)))
        left.move_to_joint_positions(position)
        rospy.sleep(2.0)
    
    # Return to start position
    print("Returning to start position...")
    left.move_to_joint_positions(start_angles)
    
    print("Wave motion completed successfully!")

if __name__ == '__main__':
    try:
        move_baxter_with_interface()
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except Exception as e:
        print("Error: {}".format(e))