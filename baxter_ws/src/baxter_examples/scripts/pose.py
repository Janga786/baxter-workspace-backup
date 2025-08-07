#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Baxter the Bartender: The Rockstar Mixologist (v5.0 - The Spectacle)
Copyright (c) 2024 - Robotics Performance Division

A high-energy, percussive performance where Baxter mixes a "mocktail"
like a rockstar drummer. This script uses pre-defined joint angles for
maximum speed and flair, interacting with real-world objects.
"""

import rospy
import cv2
import numpy as np
import random
import time
import threading

import baxter_interface
from cv_bridge import CvBridge

# --- THE CHOREOGRAPHY & SHOW CONFIGURATION ---
# This is the heart of the performance. Each entry is a "beat" in the routine.

PERFORMANCE_CONFIG = {
    # Pre-calculated joint angles for speed and reliability.
    # These need to be calibrated to your specific workspace setup.
    'poses': {
        'home': { # A ready pose, arms wide
            'left_s0': -1.4, 'left_s1': -0.2, 'left_e1': 1.6,
            'right_s0': 1.4, 'right_s1': -0.2, 'right_e1': 1.6,
        },
        'grab_cup_left': {
            'left_s0': -0.45, 'left_s1': 0.1, 'left_e0': -1.8, 'left_e1': 1.8,
            'left_w0': -0.5, 'left_w1': 1.0, 'left_w2': -0.9
        },
        'grab_shaker_right': {
            'right_s0': 0.25, 'right_s1': 0.2, 'right_e0': 1.0, 'right_e1': 1.5,
            'right_w0': 0.4, 'right_w1': 1.1, 'right_w2': 0.9
        },
        'pour_position': { # Left arm holds cup over shaker held by right arm
            'left_s0': -0.2, 'left_s1': -0.5, 'left_e0': -1.2, 'left_e1': 1.9, 'left_w1': 1.5,
            'right_s0': 0.2, 'right_s1': 0.4, 'right_e0': 0.8, 'right_e1': 1.4, 'right_w1': 1.0,
        },
        'shake_high_left': { 'left_s1': -0.8, 'left_e1': 1.5 },
        'shake_low_left': { 'left_s1': -0.5, 'left_e1': 1.8 },
        'tada': {
            'left_s0': -1.0, 'left_s1': -0.8, 'left_e1': 1.0, 'left_w2': -1.5,
            'right_s0': 1.0, 'right_s1': -0.8, 'right_e1': 1.0, 'right_w2': 1.5,
        }
    },
    
    # The sequence of actions, timed to a beat.
    'routine': [
        {'action': 'display', 'text': 'BAXTER', 'subtext': 'THE MIXOLOGIST'},
        {'action': 'move', 'pose': 'home', 'duration': 1.0},
        {'action': 'sfx', 'sound': 'start_bell'},
        
        # --- The Pickup ---
        {'action': 'move', 'pose': 'grab_cup_left', 'sfx': 'whoosh_1', 'duration': 0.5},
        {'action': 'gripper', 'limb': 'left', 'state': 'close', 'duration': 0.5},
        {'action': 'move', 'pose': 'home', 'duration': 0.5},
        {'action': 'move', 'pose': 'grab_shaker_right', 'sfx': 'whoosh_2', 'duration': 0.5},
        {'action': 'gripper', 'limb': 'right', 'state': 'close', 'duration': 0.5},
        {'action': 'move', 'pose': 'home', 'duration': 0.5},
        
        # --- The Pour ---
        {'action': 'move', 'pose': 'pour_position', 'sfx': 'clink', 'duration': 1.0},
        {'action': 'display', 'text': 'THE POUR'},
        {'action': 'pose', 'limb': 'left', 'angles': {'left_w1': 0.0}, 'sfx': 'pour', 'duration': 2.0}, # Tilt wrist to pour
        {'action': 'move', 'pose': 'home', 'sfx': 'whoosh_1', 'duration': 1.0},
        {'action': 'gripper', 'limb': 'left', 'state': 'open', 'duration': 0.5}, # Drop the first cup
        
        # --- The Shake: Percussion Solo! ---
        {'action': 'display', 'text': 'SHAKE IT!', 'subtext': 'SOUND ON!'},
        {'action': 'pose', 'limb': 'right', 'angles': {'right_s0': -0.2, 'right_s1': -0.6, 'right_e1': 1.5}, 'duration': 0.5},
        # Fast shaking sequence
        {'action': 'pose', 'limb': 'right', 'angles': {'right_s1': -0.8}, 'sfx': 'shake_1', 'duration': 0.2},
        {'action': 'pose', 'limb': 'right', 'angles': {'right_s1': -0.4}, 'sfx': 'shake_2', 'duration': 0.2},
        {'action': 'pose', 'limb': 'right', 'angles': {'right_s1': -0.8}, 'sfx': 'shake_1', 'duration': 0.2},
        {'action': 'pose', 'limb': 'right', 'angles': {'right_s1': -0.4}, 'sfx': 'shake_2', 'duration': 0.2},
        {'action': 'pose', 'limb': 'right', 'angles': {'right_e0': 1.5, 'right_w0': 1.2}, 'sfx': 'whoosh_2', 'duration': 0.4}, # Flair move
        {'action': 'pose', 'limb': 'right', 'angles': {'right_e0': 0.8, 'right_w0': 0.4}, 'sfx': 'whoosh_1', 'duration': 0.4}, # Flair back
        
        # --- The Finale ---
        {'action': 'move', 'pose': 'home', 'duration': 0.5},
        {'action': 'move', 'pose': 'grab_cup_left', 'duration': 0.5}, # Grab the empty cup
        {'action': 'gripper', 'limb': 'left', 'state': 'close', 'duration': 0.5},
        {'action': 'move', 'pose': 'pour_position', 'sfx': 'clink', 'duration': 1.0}, # Bring them together for final pour
        {'action': 'display', 'text': 'FINALE!'},
        {'action': 'pose', 'limb': 'right', 'angles': {'right_w1': 0.0}, 'sfx': 'pour_final', 'duration': 2.0},
        
        # --- The Tada! ---
        {'action': 'move', 'pose': 'home', 'sfx': 'whoosh_combo', 'duration': 1.0},
        {'action': 'gripper', 'limb': 'left', 'state': 'open', 'duration': 0.2},
        {'action': 'gripper', 'limb': 'right', 'state': 'open', 'duration': 0.2},
        {'action': 'display', 'text': 'TADA!', 'subtext': 'Thank you!'},
        {'action': 'sfx', 'sound': 'fanfare'},
        {'action': 'move', 'pose': 'tada', 'duration': 4.0},
    ]
}

class RockstarMixologist(object):

    def __init__(self, config):
        self.config = config
        self.bridge = CvBridge()
        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')
        self.left_gripper = baxter_interface.Gripper('left')
        self.right_gripper = baxter_interface.Gripper('right')
        self.display_pub = rospy.Publisher('/robot/xdisplay', baxter_interface.camera.Image, latch=True, queue_size=1)
        
        # Calibrate grippers
        if not self.left_gripper.calibrated(): self.left_gripper.calibrate()
        if not self.right_gripper.calibrated(): self.right_gripper.calibrate()
        self.left_gripper.set_holding_force(100.0)
        self.right_gripper.set_holding_force(100.0)
        
    def _draw_visualizer(self, title="", subtitle=""):
        """Draws a dynamic, flashy 'music visualizer' style screen."""
        frame = np.zeros((600, 1024, 3), dtype=np.uint8)
        # Add some random "stars" for a dynamic background
        for _ in range(50):
            x, y = random.randint(0, 1023), random.randint(0, 599)
            size = random.randint(1, 4)
            color = random.choice([(255,0,0), (0,255,0), (0,0,255), (255,255,0), (255,0,255)])
            cv2.circle(frame, (x, y), size, color, -1)
        
        # Big, bold text
        cv2.putText(frame, title, (512 - len(title) * 25, 280), cv2.FONT_HERSHEY_COMPLEX, 2.5, (255, 255, 255), 8)
        cv2.putText(frame, subtitle, (512 - len(subtitle) * 12, 400), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (200, 200, 200), 4)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.display_pub.publish(msg)

    def _sfx(self, sound):
        print(f"  SFX: Playing '{sound}'")

    def run_performance(self):
        """Executes the entire choreographed routine."""
        print("Rockstar Mixologist Performance: STARTING!")
        
        # Set a faster joint speed
        self.left_arm.set_joint_position_speed(0.8)
        self.right_arm.set_joint_position_speed(0.8)

        for step in self.config['routine']:
            if rospy.is_shutdown(): break
            
            action = step['action']
            duration = step.get('duration', 0.1)
            
            # Play sound effect if it exists for this step
            if 'sfx' in step:
                self._sfx(step['sfx'])
            
            # --- ACTION HANDLER ---
            if action == 'display':
                self._draw_visualizer(step['text'], step.get('subtext', ''))
            elif action == 'move':
                pose = self.config['poses'][step['pose']]
                self.left_arm.move_to_joint_positions({k:v for k,v in pose.items() if k.startswith('left')}, timeout=5.0)
                self.right_arm.move_to_joint_positions({k:v for k,v in pose.items() if k.startswith('right')}, timeout=5.0)
            elif action == 'pose':
                limb = self.left_arm if step['limb'] == 'left' else self.right_arm
                limb.move_to_joint_positions(step['angles'], timeout=2.0)
            elif action == 'gripper':
                gripper = self.left_gripper if step['limb'] == 'left' else self.right_gripper
                if step['state'] == 'close':
                    gripper.close()
                else:
                    gripper.open()

            time.sleep(duration)
        
        print("Performance FINISHED!")

if __name__ == '__main__':
    rospy.init_node('baxter_rockstar_mixologist')
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    
    show = RockstarMixologist(PERFORMANCE_CONFIG)
    try:
        show.run_performance()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("\nShow's over. Returning to neutral.")
        baxter_interface.Limb('left').move_to_neutral()
        baxter_interface.Limb('right').move_to_neutral()
        rs.disable()