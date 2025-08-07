#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Baxter Demo Utilities
Helper functions for Fort Lewis College Demo
Python 2.7 Compatible
"""

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import tf
import numpy as np
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import sys

class SafetyMonitor(object):
    """Monitor robot safety during demonstrations"""
    
    def __init__(self):
        self.collision_threshold = 5.0  # Force threshold in Nm
        self.velocity_threshold = 0.5   # Joint velocity threshold
        self._limb_left = baxter_interface.Limb('left')
        self._limb_right = baxter_interface.Limb('right')
        
    def check_collision(self):
        """Check for potential collisions using joint torques"""
        left_torques = self._limb_left.joint_efforts()
        right_torques = self._limb_right.joint_efforts()
        
        # Check if any torque exceeds threshold
        for joint, torque in left_torques.items():
            if abs(torque) > self.collision_threshold:
                rospy.logwarn("High torque detected on %s: %.2f Nm" % (joint, torque))
                return True
                
        for joint, torque in right_torques.items():
            if abs(torque) > self.collision_threshold:
                rospy.logwarn("High torque detected on %s: %.2f Nm" % (joint, torque))
                return True
                
        return False
    
    def check_velocity_limits(self):
        """Ensure joint velocities are within safe limits"""
        left_vels = self._limb_left.joint_velocities()
        right_vels = self._limb_right.joint_velocities()
        
        for joint, vel in left_vels.items():
            if abs(vel) > self.velocity_threshold:
                rospy.logwarn("High velocity on %s: %.2f rad/s" % (joint, vel))
                return False
                
        for joint, vel in right_vels.items():
            if abs(vel) > self.velocity_threshold:
                rospy.logwarn("High velocity on %s: %.2f rad/s" % (joint, vel))
                return False
                
        return True

class DemoPositions(object):
    """Pre-defined positions for reliable demonstrations"""
    
    # Safe home position
    HOME_LEFT = {
        'left_s0': -0.08, 'left_s1': -1.0, 'left_e0': -1.19,
        'left_e1': 1.94, 'left_w0': 0.67, 'left_w1': 1.03,
        'left_w2': -0.50
    }
    
    HOME_RIGHT = {
        'right_s0': 0.08, 'right_s1': -1.0, 'right_e0': 1.19,
        'right_e1': 1.94, 'right_w0': -0.67, 'right_w1': 1.03,
        'right_w2': 0.50
    }
    
    # High-five position
    HIGH_FIVE_RIGHT = {
        'right_s0': -0.358, 'right_s1': -0.759, 'right_e0': 1.831,
        'right_e1': 1.040, 'right_w0': -0.668, 'right_w1': 1.031,
        'right_w2': 0.498
    }
    
    # Object pickup positions
    PICKUP_APPROACH = {
        'right_s0': -0.461, 'right_s1': -0.202, 'right_e0': 1.807,
        'right_e1': 1.507, 'right_w0': -1.545, 'right_w1': 1.457,
        'right_w2': -0.085
    }
    
    PICKUP_RETREAT = {
        'right_s0': -0.461, 'right_s1': -0.702, 'right_e0': 1.807,
        'right_e1': 1.507, 'right_w0': -1.545, 'right_w1': 1.457,
        'right_w2': -0.085
    }

def smooth_joint_trajectory(start_angles, end_angles, steps=10):
    """Generate smooth trajectory between joint positions"""
    trajectory = []
    
    for i in range(steps + 1):
        t = float(i) / steps
        # Use cubic interpolation for smooth acceleration/deceleration
        t = t * t * (3.0 - 2.0 * t)
        
        interpolated = {}
        for joint in start_angles:
            if joint in end_angles:
                interpolated[joint] = start_angles[joint] + \
                                    t * (end_angles[joint] - start_angles[joint])
        
        trajectory.append(interpolated)
    
    return trajectory

def validate_joint_angles(joint_angles):
    """Validate joint angles are within Baxter's limits"""
    # Baxter joint limits (approximate)
    joint_limits = {
        's0': (-1.7016, 1.7016),
        's1': (-2.147, 1.047),
        'e0': (-3.0541, 3.0541),
        'e1': (-0.05, 2.618),
        'w0': (-3.059, 3.059),
        'w1': (-1.5707, 2.094),
        'w2': (-3.059, 3.059)
    }
    
    for joint, angle in joint_angles.items():
        joint_type = joint.split('_')[1]
        if joint_type in joint_limits:
            min_limit, max_limit = joint_limits[joint_type]
            if not (min_limit <= angle <= max_limit):
                rospy.logwarn("Joint %s angle %.2f exceeds limits [%.2f, %.2f]" % 
                            (joint, angle, min_limit, max_limit))
                return False
    
    return True

def create_pose_stamped(x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
    """Create a PoseStamped message"""
    pose = PoseStamped()
    pose.header.frame_id = "base"
    pose.header.stamp = rospy.Time.now()
    
    # Position
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    
    # Orientation (convert RPY to quaternion)
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]
    
    return pose

class ErrorRecovery(object):
    """Handle common demo errors gracefully"""
    
    def __init__(self, left_arm, right_arm):
        self._left = left_arm
        self._right = right_arm
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        
    def handle_joint_stuck(self, limb_name):
        """Handle stuck joint by gentle wiggling"""
        rospy.logwarn("Attempting to free stuck joint on %s arm" % limb_name)
        
        limb = self._left if limb_name == 'left' else self._right
        current_pos = limb.joint_angles()
        
        # Small oscillations to free joint
        for _ in range(3):
            for joint, angle in current_pos.items():
                wiggle_pos = current_pos.copy()
                wiggle_pos[joint] = angle + 0.05
                limb.set_joint_positions(wiggle_pos)
                rospy.sleep(0.1)
                wiggle_pos[joint] = angle - 0.05
                limb.set_joint_positions(wiggle_pos)
                rospy.sleep(0.1)
        
        return True
    
    def handle_gripper_error(self, gripper_name):
        """Handle gripper errors"""
        rospy.logwarn("Handling gripper error on %s" % gripper_name)
        
        gripper = baxter_interface.Gripper(gripper_name, CHECK_VERSION)
        
        # Try recalibration
        try:
            gripper.calibrate()
            return True
        except:
            rospy.logerr("Gripper recalibration failed")
            return False
    
    def emergency_stop_recovery(self):
        """Recover from emergency stop"""
        rospy.loginfo("Attempting emergency stop recovery...")
        
        # Reset enable state
        try:
            self._rs.reset()
            rospy.sleep(1.0)
            self._rs.enable()
            return True
        except:
            rospy.logerr("Failed to recover from emergency stop")
            return False

def play_success_animation(face, arm):
    """Play success animation"""
    # Face shows very happy
    face.show_very_happy()
    
    # Arm does small celebration
    current = arm.joint_angles()
    celebrate = current.copy()
    celebrate[arm.name + '_w2'] = current[arm.name + '_w2'] + 0.3
    
    for _ in range(2):
        arm.move_to_joint_positions(celebrate, timeout=0.5)
        arm.move_to_joint_positions(current, timeout=0.5)

def estimate_demo_timing():
    """Return estimated timing for each demo phase"""
    return {
        'phase_1_whiteboard': 30,      # 30 seconds
        'phase_2_teaching': 90,        # 90 seconds
        'phase_3_sorting': 90,         # 90 seconds
        'phase_4_personality': 30,     # 30 seconds
        'total': 240                   # 4 minutes total
    }

class PerformanceMonitor(object):
    """Monitor demo performance metrics"""
    
    def __init__(self):
        self.start_time = None
        self.phase_times = {}
        self.error_count = 0
        self.interaction_count = 0
        
    def start_demo(self):
        """Mark demo start"""
        self.start_time = rospy.Time.now()
        rospy.loginfo("Demo started at %s" % self.start_time)
        
    def mark_phase(self, phase_name):
        """Mark phase completion"""
        if self.start_time:
            elapsed = (rospy.Time.now() - self.start_time).to_sec()
            self.phase_times[phase_name] = elapsed
            rospy.loginfo("Phase '%s' completed at %.1f seconds" % (phase_name, elapsed))
    
    def record_error(self, error_type):
        """Record error occurrence"""
        self.error_count += 1
        rospy.logwarn("Error recorded: %s (total errors: %d)" % (error_type, self.error_count))
    
    def record_interaction(self):
        """Record successful interaction"""
        self.interaction_count += 1
        
    def print_summary(self):
        """Print performance summary"""
        if self.start_time:
            total_time = (rospy.Time.now() - self.start_time).to_sec()
            
            rospy.loginfo("\n=== DEMO PERFORMANCE SUMMARY ===")
            rospy.loginfo("Total Duration: %.1f seconds" % total_time)
            rospy.loginfo("Phases Completed: %d" % len(self.phase_times))
            rospy.loginfo("Total Errors: %d" % self.error_count)
            rospy.loginfo("Successful Interactions: %d" % self.interaction_count)
            
            rospy.loginfo("\nPhase Timing:")
            for phase, time in sorted(self.phase_times.items()):
                rospy.loginfo("  %s: %.1f seconds" % (phase, time))
            
            # Performance grade
            if self.error_count == 0 and len(self.phase_times) >= 4:
                rospy.loginfo("\nPerformance Grade: EXCELLENT")
            elif self.error_count <= 2:
                rospy.loginfo("\nPerformance Grade: GOOD")
            else:
                rospy.loginfo("\nPerformance Grade: NEEDS IMPROVEMENT")

# Demo configuration parameters
DEMO_CONFIG = {
    'speed_factor': 0.3,           # Speed multiplier for safety
    'gripper_force': 30.0,         # Gripper force in Newtons
    'position_tolerance': 0.01,    # Position accuracy in radians
    'timeout_multiplier': 1.5,     # Timeout safety factor
    'enable_collision_check': True,
    'enable_face_tracking': True,
    'audience_interaction': True,
    'fallback_enabled': True
}

def load_demo_config(config_file=None):
    """Load demo configuration from file or defaults"""
    if config_file:
        try:
            import yaml
            with open(config_file, 'r') as f:
                custom_config = yaml.load(f)
                DEMO_CONFIG.update(custom_config)
                rospy.loginfo("Loaded custom config from %s" % config_file)
        except:
            rospy.logwarn("Failed to load config file, using defaults")
    
    return DEMO_CONFIG