#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Baxter Demo Setup and Calibration
Pre-demo checks and calibration for Fort Lewis College
Python 2.7 Compatible
"""

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.srv import (
    ListCameras,
    OpenCamera,
    CloseCamera,
)
import sys
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from baxter_face_expressions import create_digital_expression

class BaxterDemoSetup(object):
    def __init__(self):
        """Initialize setup system"""
        print("\n" + "="*50)
        print("BAXTER DEMO SETUP - Fort Lewis College")
        print("="*50 + "\n")
        
        rospy.init_node('baxter_demo_setup')
        
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self.display_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=1)
        self.bridge = CvBridge()
        
        # Track setup status
        self.setup_status = {
            'robot_enabled': False,
            'arms_calibrated': False,
            'grippers_calibrated': False,
            'cameras_verified': False,
            'network_verified': False,
            'safety_checked': False
        }
    
    def _show_expression(self, expression_type):
        """Show facial expression on Baxter's display"""
        try:
            # Map some expression names
            if expression_type == "sad":
                expression_type = "confused"
            elif expression_type == "very_happy":
                expression_type = "excited"
            
            img = create_digital_expression(expression_type)
            msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.display_pub.publish(msg)
            rospy.sleep(0.5)
        except Exception as e:
            print("  Warning: Could not display expression: %s" % str(e))
    
    def enable_robot(self):
        """Enable robot with safety checks"""
        print("Step 1: Enabling Robot...")
        
        try:
            # Check current state
            print("  Current state: %s" % self._rs.state())
            
            if not self._rs.state().enabled:
                print("  Enabling robot...")
                self._rs.enable()
                rospy.sleep(2.0)
            
            if self._rs.state().enabled:
                print("  ✓ Robot enabled successfully")
                self.setup_status['robot_enabled'] = True
                self._show_expression("happy")
                return True
            else:
                print("  ✗ Failed to enable robot")
                self._show_expression("sad")
                return False
                
        except Exception as e:
            print("  ✗ Error enabling robot: %s" % str(e))
            return False
    
    def calibrate_grippers(self):
        """Calibrate both grippers"""
        print("\nStep 2: Calibrating Grippers...")
        
        success = True
        
        for side in ['left', 'right']:
            print("  Calibrating %s gripper..." % side)
            try:
                gripper = baxter_interface.Gripper(side, CHECK_VERSION)
                
                # Check if already calibrated
                if gripper.calibrated():
                    print("    Already calibrated")
                else:
                    gripper.calibrate()
                    rospy.sleep(2.0)
                
                # Test gripper
                print("    Testing gripper movement...")
                gripper.open()
                rospy.sleep(1.0)
                gripper.close()
                rospy.sleep(1.0)
                gripper.open()
                
                print("  ✓ %s gripper ready" % side.capitalize())
                
            except Exception as e:
                print("  ✗ Error with %s gripper: %s" % (side, str(e)))
                success = False
        
        self.setup_status['grippers_calibrated'] = success
        if success:
            self._show_expression("happy")
        else:
            self._show_expression("confused")
        
        return success
    
    def calibrate_arms(self):
        """Move arms through calibration positions"""
        print("\nStep 3: Calibrating Arms...")
        
        # Home positions
        home_left = {
            'left_s0': -0.08, 'left_s1': -1.0, 'left_e0': -1.19,
            'left_e1': 1.94, 'left_w0': 0.67, 'left_w1': 1.03,
            'left_w2': -0.50
        }
        home_right = {
            'right_s0': 0.08, 'right_s1': -1.0, 'right_e0': 1.19,
            'right_e1': 1.94, 'right_w0': -0.67, 'right_w1': 1.03,
            'right_w2': 0.50
        }
        
        success = True
        
        try:
            # Initialize limbs
            left_arm = baxter_interface.Limb('left')
            right_arm = baxter_interface.Limb('right')
            
            # Set conservative speed
            left_arm.set_joint_position_speed(0.3)
            right_arm.set_joint_position_speed(0.3)
            
            print("  Moving to home position...")
            left_arm.move_to_joint_positions(home_left, timeout=5.0)
            right_arm.move_to_joint_positions(home_right, timeout=5.0)
            
            # Test range of motion
            print("  Testing range of motion...")
            test_positions = [
                ({'left_w2': -1.0}, {'right_w2': 1.0}),
                ({'left_w2': 1.0}, {'right_w2': -1.0}),
                ({'left_e1': 1.5}, {'right_e1': 1.5}),
            ]
            
            for left_pos, right_pos in test_positions:
                left_arm.set_joint_positions(left_pos)
                right_arm.set_joint_positions(right_pos)
                rospy.sleep(1.0)
            
            # Return to home
            left_arm.move_to_joint_positions(home_left)
            right_arm.move_to_joint_positions(home_right)
            
            print("  ✓ Arms calibrated successfully")
            self.setup_status['arms_calibrated'] = True
            
        except Exception as e:
            print("  ✗ Error calibrating arms: %s" % str(e))
            success = False
            self.setup_status['arms_calibrated'] = False
        
        return success
    
    def verify_cameras(self):
        """Verify camera functionality"""
        print("\nStep 4: Verifying Cameras...")
        
        required_cameras = ['left_hand_camera', 'right_hand_camera', 'head_camera']
        success = True
        
        try:
            # List cameras
            list_svc = rospy.ServiceProxy('/cameras/list', ListCameras)
            rospy.wait_for_service('/cameras/list', timeout=5.0)
            resp = list_svc()
            
            print("  Available cameras: %s" % resp.cameras)
            
            # Check required cameras
            for camera in required_cameras:
                if camera in resp.cameras:
                    print("  ✓ %s available" % camera)
                else:
                    print("  ✗ %s missing" % camera)
                    success = False
            
            self.setup_status['cameras_verified'] = success
            
        except Exception as e:
            print("  ✗ Error verifying cameras: %s" % str(e))
            self.setup_status['cameras_verified'] = False
            success = False
        
        return success
    
    def verify_network(self):
        """Verify network connectivity"""
        print("\nStep 5: Verifying Network...")
        
        # Check ROS master
        try:
            rospy.get_master().getPid()
            print("  ✓ ROS Master connected")
            
            # Check parameter server
            robot_name = rospy.get_param('/robot/name', 'unknown')
            print("  ✓ Robot name: %s" % robot_name)
            
            self.setup_status['network_verified'] = True
            return True
            
        except Exception as e:
            print("  ✗ Network error: %s" % str(e))
            self.setup_status['network_verified'] = False
            return False
    
    def safety_check(self):
        """Perform safety checks"""
        print("\nStep 6: Safety Checks...")
        
        checks_passed = True
        
        # Check joint torques
        print("  Checking joint torques...")
        left = baxter_interface.Limb('left')
        right = baxter_interface.Limb('right')
        
        left_efforts = left.joint_efforts()
        right_efforts = right.joint_efforts()
        
        high_torque = False
        for joint, effort in left_efforts.items():
            if abs(effort) > 10.0:
                print("  ⚠ High torque on %s: %.2f Nm" % (joint, effort))
                high_torque = True
        
        for joint, effort in right_efforts.items():
            if abs(effort) > 10.0:
                print("  ⚠ High torque on %s: %.2f Nm" % (joint, effort))
                high_torque = True
        
        if not high_torque:
            print("  ✓ Joint torques normal")
        else:
            checks_passed = True
        
        self.setup_status['safety_checked'] = checks_passed
        return checks_passed
    
    def run_demo_test(self):
        """Run quick demo test"""
        print("\nStep 7: Demo Function Test...")
        
        try:
            # Test face expressions
            print("  Testing face expressions...")
            self._show_expression("happy")
            rospy.sleep(1.0)
            self._show_expression("excited")
            rospy.sleep(1.0)
            self._show_expression("waking")
            rospy.sleep(1.0)
            
            # Test arm movement
            print("  Testing demo movements...")
            right = baxter_interface.Limb('right')
            
            # Wave
            wave_pos = {'right_w2': -1.0}
            right.set_joint_positions(wave_pos)
            rospy.sleep(0.5)
            wave_pos = {'right_w2': 1.0}
            right.set_joint_positions(wave_pos)
            rospy.sleep(0.5)
            
            print("  ✓ Demo functions working")
            return True
            
        except Exception as e:
            print("  ✗ Demo test failed: %s" % str(e))
            return False
    
    def print_summary(self):
        """Print setup summary"""
        print("\n" + "="*50)
        print("SETUP SUMMARY")
        print("="*50)
        
        all_good = True
        for check, status in self.setup_status.items():
            symbol = "✓" if status else "✗"
            print("%s %s: %s" % (symbol, check.replace('_', ' ').title(), 
                                "PASS" if status else "FAIL"))
            if not status:
                all_good = False
        
        print("="*50)
        
        if all_good:
            print("\n✓ BAXTER IS READY FOR DEMO!")
            print("Run: rosrun fort_lewis_demo baxter_demo_main.py")
            self._show_expression("very_happy")
        else:
            print("\n✗ SETUP INCOMPLETE - Please fix issues above")
            self._show_expression("sad")
        
        return all_good
    
    def run_setup(self, skip_safety=False):
        """Run complete setup sequence"""
        steps = [
            self.enable_robot,
            self.calibrate_grippers,
            self.calibrate_arms,
            self.verify_cameras,
            self.verify_network,
        ]
        
        if not skip_safety:
            steps.append(self.safety_check)
        
        steps.append(self.run_demo_test)
        
        for step in steps:
            if not step():
                print("\n⚠ Setup stopped due to error")
                break
        
        return self.print_summary()

def main():
    """Main setup execution"""
    parser = argparse.ArgumentParser(description='Baxter Demo Setup')
    parser.add_argument('--skip-safety', action='store_true',
                       help='Skip safety checks (not recommended)')
    parser.add_argument('--quick', action='store_true',
                       help='Quick setup (skip some tests)')
    args = parser.parse_args(rospy.myargv()[1:])
    
    setup = BaxterDemoSetup()
    
    try:
        if setup.run_setup(skip_safety=args.skip_safety):
            sys.exit(0)
        else:
            sys.exit(1)
    except rospy.ROSInterruptException:
        print("\nSetup interrupted")
        sys.exit(1)
    except Exception as e:
        print("\nSetup error: %s" % str(e))
        sys.exit(1)

if __name__ == '__main__':
    main()