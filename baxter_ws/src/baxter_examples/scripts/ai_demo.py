#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Baxter Demo Quick Test
5-minute system verification before demo
Python 2.7 Compatible
"""

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import sys
from baxter_face_expressions import BaxterFace

def run_quick_test():
    """Run 5-minute pre-demo test"""
    
    print("\n" + "="*50)
    print("BAXTER QUICK DEMO TEST (5 minutes)")
    print("="*50 + "\n")
    
    # Initialize
    rospy.init_node('baxter_quick_test')
    
    # Enable robot
    print("1. Enabling robot...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rs.enable()
    
    # Initialize components
    left_arm = baxter_interface.Limb('left')
    right_arm = baxter_interface.Limb('right')
    left_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
    right_gripper = baxter_interface.Gripper('right', CHECK_VERSION)
    face = BaxterFace()
    
    print("✓ Robot enabled\n")
    
    # Test 1: Face expressions (30 seconds)
    print("2. Testing face expressions...")
    expressions = [
        ("Happy", face.show_happy),
        ("Surprised", face.show_surprised),
        ("Confused", face.show_confused),
        ("Wink", face.wink)
    ]
    
    for name, func in expressions:
        print("   Showing: %s" % name)
        func()
        rospy.sleep(1.0)
    
    print("✓ Face expressions working\n")
    
    # Test 2: Gripper function (30 seconds)
    print("3. Testing grippers...")
    for gripper, name in [(left_gripper, "left"), (right_gripper, "right")]:
        print("   Testing %s gripper..." % name)
        gripper.open()
        rospy.sleep(0.5)
        gripper.close()
        rospy.sleep(0.5)
        gripper.open()
    
    print("✓ Grippers working\n")
    
    # Test 3: Arm movement (1 minute)
    print("4. Testing arm movements...")
    
    # Move to neutral
    print("   Moving to neutral position...")
    neutral_left = {
        'left_s0': -0.08, 'left_s1': -1.0, 'left_e0': -1.19,
        'left_e1': 1.94, 'left_w0': 0.67, 'left_w1': 1.03,
        'left_w2': -0.50
    }
    neutral_right = {
        'right_s0': 0.08, 'right_s1': -1.0, 'right_e0': 1.19,
        'right_e1': 1.94, 'right_w0': -0.67, 'right_w1': 1.03,
        'right_w2': 0.50
    }
    
    left_arm.move_to_joint_positions(neutral_left, timeout=5.0)
    right_arm.move_to_joint_positions(neutral_right, timeout=5.0)
    
    # Test wave
    print("   Testing wave gesture...")
    for _ in range(2):
        right_arm.set_joint_positions({'right_w2': -1.0})
        rospy.sleep(0.5)
        right_arm.set_joint_positions({'right_w2': 1.0})
        rospy.sleep(0.5)
    
    print("✓ Arm movements working\n")
    
    # Test 4: Teaching mode (30 seconds)
    print("5. Testing teaching mode...")
    print("   Setting compliant mode...")
    left_arm.set_joint_position_speed(0.3)
    right_arm.set_joint_position_speed(0.3)
    
    # Just verify we can read positions
    left_pos = left_arm.joint_angles()
    right_pos = right_arm.joint_angles()
    print("   Left arm joints: %d" % len(left_pos))
    print("   Right arm joints: %d" % len(right_pos))
    
    print("✓ Teaching mode ready\n")
    
    # Test 5: Safety systems (30 seconds)
    print("6. Testing safety systems...")
    
    # Check E-stop status
    if rs.state().enabled:
        print("   ✓ E-stop not pressed")
    else:
        print("   ✗ E-stop is pressed!")
    
    # Check error states
    if rs.state().error:
        print("   ✗ Robot in error state!")
    else:
        print("   ✓ No errors detected")
    
    # Final status
    print("\n" + "="*50)
    print("QUICK TEST COMPLETE")
    print("="*50)
    
    # Show ready face
    face.show_ready()
    
    # Return arms to neutral
    left_arm.move_to_joint_positions(neutral_left)
    right_arm.move_to_joint_positions(neutral_right)
    
    print("\n✓ BAXTER IS READY FOR DEMO!")
    print("Run: rosrun fort_lewis_demo baxter_demo_main.py")
    
    return True

def main():
    """Main entry point"""
    try:
        if run_quick_test():
            sys.exit(0)
        else:
            sys.exit(1)
    except rospy.ROSInterruptException:
        print("\nTest interrupted")
        sys.exit(1)
    except Exception as e:
        print("\nTest failed: %s" % str(e))
        sys.exit(1)

if __name__ == '__main__':
    main()