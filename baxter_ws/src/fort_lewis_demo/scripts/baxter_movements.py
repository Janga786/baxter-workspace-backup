#!/usr/bin/env python2.7

"""
Baxter Movement Library
Extracted movement functions for the master demo
"""

import rospy
import baxter_interface
from baxter_interface import Head, Gripper
import time
import math


class BaxterMovements(object):
    """Library of common Baxter movements and gestures"""
    
    def __init__(self):
        """Initialize movement controller"""
        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')
        self.left_gripper = Gripper('left')
        self.right_gripper = Gripper('right')
        self.head = Head()
        
        # Set safe movement speeds
        self.left_arm.set_joint_position_speed(0.3)
        self.right_arm.set_joint_position_speed(0.3)
    
    def safe_move_arm(self, arm, target_positions, move_speed=0.3):
        """Safely move arm to target position with error handling"""
        try:
            arm.set_joint_position_speed(move_speed)
            arm.move_to_joint_positions(target_positions)
            return True
        except Exception as e:
            rospy.logerr("Arm movement error: {}".format(e))
            return False
    
    def move_to_neutral(self):
        """Move to safe neutral position"""
        neutral_left = {
            'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0,
            'left_e0': -0.7, 'left_e1': 1.5, 'left_s0': 0.0, 'left_s1': -0.3
        }
        neutral_right = {
            'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0,
            'right_e0': 0.7, 'right_e1': 1.5, 'right_s0': 0.0, 'right_s1': -0.3
        }
        
        self.safe_move_arm(self.left_arm, neutral_left)
        self.safe_move_arm(self.right_arm, neutral_right)
        self.head.set_pan(0.0)
        
        # Close grippers
        self.left_gripper.close()
        self.right_gripper.close()
    
    def wave_gesture(self, limb_name='right', waves=3):
        """Perform waving motion with specified arm"""
        limb = self.left_arm if limb_name == 'left' else self.right_arm
        current_pos = limb.joint_angles()
        
        # Create wave motion
        wave_joint = limb_name + '_w2'
        original_angle = current_pos[wave_joint]
        
        for _ in range(waves):
            # Wave up
            current_pos[wave_joint] = original_angle + 0.8
            self.safe_move_arm(limb, current_pos, 0.5)
            time.sleep(0.3)
            
            # Wave down
            current_pos[wave_joint] = original_angle - 0.8
            self.safe_move_arm(limb, current_pos, 0.5)
            time.sleep(0.3)
        
        # Return to original position
        current_pos[wave_joint] = original_angle
        self.safe_move_arm(limb, current_pos)
    
    def high_five_position(self, limb_name='right'):
        """Move arm to high-five position"""
        if limb_name == 'right':
            high_five_pos = {
                'right_s0': -0.358, 'right_s1': -0.759, 'right_e0': 1.831,
                'right_e1': 1.040, 'right_w0': -0.668, 'right_w1': 1.031,
                'right_w2': 0.498
            }
            return self.safe_move_arm(self.right_arm, high_five_pos)
        else:
            high_five_pos = {
                'left_s0': 0.358, 'left_s1': -0.759, 'left_e0': -1.831,
                'left_e1': 1.040, 'left_w0': 0.668, 'left_w1': 1.031,
                'left_w2': -0.498
            }
            return self.safe_move_arm(self.left_arm, high_five_pos)
    
    def celebration_dance(self):
        """Perform a celebration dance sequence"""
        # Get current positions
        current_right = self.right_arm.joint_angles()
        current_left = self.left_arm.joint_angles()
        
        # Store originals
        original_positions = {
            'right_s1': current_right['right_s1'],
            'left_s1': current_left['left_s1'],
            'right_e1': current_right['right_e1'],
            'left_e1': current_left['left_e1']
        }
        
        # Dance sequence
        dance_moves = [
            {'right_s1': 0.5, 'left_s1': -0.5, 'right_e1': 0.3, 'left_e1': 0.3},
            {'right_s1': -0.5, 'left_s1': 0.5, 'right_e1': -0.3, 'left_e1': -0.3},
            {'right_s1': 0.3, 'left_s1': -0.3, 'right_e1': 0.2, 'left_e1': 0.2},
            {'right_s1': -0.3, 'left_s1': 0.3, 'right_e1': -0.2, 'left_e1': -0.2},
            {'right_s1': 0.0, 'left_s1': 0.0, 'right_e1': 0.0, 'left_e1': 0.0}
        ]
        
        for i, move in enumerate(dance_moves):
            # Apply offsets to original positions
            for joint, offset in move.items():
                if 'right' in joint:
                    current_right[joint] = original_positions[joint] + offset
                else:
                    current_left[joint] = original_positions[joint] + offset
            
            # Move arms
            speed = 0.6 if i < 4 else 0.4
            self.safe_move_arm(self.right_arm, current_right, speed)
            self.safe_move_arm(self.left_arm, current_left, speed)
            time.sleep(0.4)
    
    def head_nod(self, nods=3):
        """Perform head nodding motion"""
        try:
            for _ in range(nods):
                self.head.command_nod()
                time.sleep(1.0)
        except:
            # Manual nod fallback
            for _ in range(nods):
                self.head.set_pan(0.2)
                time.sleep(0.3)
                self.head.set_pan(-0.2)
                time.sleep(0.3)
            self.head.set_pan(0.0)
    
    def look_around(self):
        """Look around at the audience"""
        look_positions = [-0.7, 0.0, 0.7, 0.0]
        
        for position in look_positions:
            self.head.set_pan(position)
            time.sleep(1.5)
    
    def stretching_sequence(self):
        """Perform stretching motion like waking up"""
        stretch_left = {
            'left_w0': -0.5, 'left_w1': -0.5, 'left_w2': 0.0,
            'left_e0': -0.8, 'left_e1': 1.2, 'left_s0': 0.5, 'left_s1': -0.3
        }
        stretch_right = {
            'right_w0': 0.5, 'right_w1': -0.5, 'right_w2': 0.0,
            'right_e0': 0.8, 'right_e1': 1.2, 'right_s0': -0.5, 'right_s1': -0.3
        }
        
        self.safe_move_arm(self.left_arm, stretch_left)
        self.safe_move_arm(self.right_arm, stretch_right)
        time.sleep(2.0)
    
    def bow_gesture(self):
        """Perform a polite bow"""
        bow_left = {
            'left_w0': 0.0, 'left_w1': 1.0, 'left_w2': 0.0,
            'left_e0': -0.5, 'left_e1': 1.8, 'left_s0': 0.3, 'left_s1': 0.2
        }
        bow_right = {
            'right_w0': 0.0, 'right_w1': 1.0, 'right_w2': 0.0,
            'right_e0': 0.5, 'right_e1': 1.8, 'right_s0': -0.3, 'right_s1': 0.2
        }
        
        self.safe_move_arm(self.left_arm, bow_left)
        self.safe_move_arm(self.right_arm, bow_right)
        time.sleep(2.0)
        
        # Return to neutral
        self.move_to_neutral()