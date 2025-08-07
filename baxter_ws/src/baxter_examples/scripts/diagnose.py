#!/usr/bin/env python

import rospy
import baxter_interface

def diagnose_baxter():
    """Comprehensive Baxter joint diagnostics"""
    
    rospy.init_node('baxter_diagnostics')
    
    print("=== BAXTER JOINT DIAGNOSTICS ===\n")
    
    # Check robot state
    try:
        rs = baxter_interface.RobotEnable()
        print("Robot State:")
        print("  Enabled:", rs.state().enabled)
        print("  Stopped:", rs.state().stopped)
        print("  Error:", rs.state().error)
        print("  Estop Button:", rs.state().estop_button)
        print("  Estop Source:", rs.state().estop_source)
        print()
    except Exception as e:
        print("ERROR: Could not get robot state:", e)
        return
    
    # Initialize limbs
    try:
        limbR = baxter_interface.Limb('right')
        limbL = baxter_interface.Limb('left')
        print("Limb interfaces initialized successfully\n")
    except Exception as e:
        print("ERROR: Could not initialize limbs:", e)
        return
    
    # Check current joint angles
    print("CURRENT JOINT POSITIONS:")
    print("Right arm:")
    right_angles = limbR.joint_angles()
    for joint, angle in right_angles.items():
        print("  {}: {:.3f} rad ({:.1f} deg)".format(joint, angle, angle * 57.2958))
    
    print("\nLeft arm:")
    left_angles = limbL.joint_angles()
    for joint, angle in left_angles.items():
        print("  {}: {:.3f} rad ({:.1f} deg)".format(joint, angle, angle * 57.2958))
    
    # Check joint velocities
    print("\nCURRENT JOINT VELOCITIES:")
    print("Right arm:")
    right_velocities = limbR.joint_velocities()
    for joint, vel in right_velocities.items():
        print("  {}: {:.3f} rad/s".format(joint, vel))
    
    print("\nLeft arm:")
    left_velocities = limbL.joint_velocities()
    for joint, vel in left_velocities.items():
        print("  {}: {:.3f} rad/s".format(joint, vel))
    
    # Check joint efforts
    print("\nCURRENT JOINT EFFORTS:")
    print("Right arm:")
    right_efforts = limbR.joint_efforts()
    for joint, effort in right_efforts.items():
        print("  {}: {:.3f} Nm".format(joint, effort))
    
    print("\nLeft arm:")
    left_efforts = limbL.joint_efforts()
    for joint, effort in left_efforts.items():
        print("  {}: {:.3f} Nm".format(joint, effort))
    
    # Test small movements
    print("\n=== MOVEMENT CAPABILITY TEST ===")
    
    def test_joint_movement(limb, joint_name, increment=0.1):
        """Test if a single joint can move"""
        try:
            current_angles = limb.joint_angles()
            target_angles = current_angles.copy()
            target_angles[joint_name] += increment
            
            print("Testing {} movement...".format(joint_name))
            limb.set_joint_position_speed(0.1)
            result = limb.move_to_joint_positions(target_angles, timeout=5.0)
            
            if result:
                print("  SUCCESS: {} moved successfully".format(joint_name))
                # Return to original position
                limb.move_to_joint_positions(current_angles, timeout=5.0)
                return True
            else:
                print("  TIMEOUT: {} movement timed out".format(joint_name))
                return False
                
        except Exception as e:
            print("  ERROR: {} movement failed - {}".format(joint_name, e))
            return False
    
    # Test each joint individually
    print("\nTesting right arm joints:")
    right_working_joints = []
    for joint in right_angles.keys():
        if test_joint_movement(limbR, joint, 0.05):  # Very small movement
            right_working_joints.append(joint)
        rospy.sleep(0.5)
    
    print("\nTesting left arm joints:")
    left_working_joints = []
    for joint in left_angles.keys():
        if test_joint_movement(limbL, joint, 0.05):  # Very small movement
            left_working_joints.append(joint)
        rospy.sleep(0.5)
    
    # Summary
    print("\n=== DIAGNOSTIC SUMMARY ===")
    print("Working right arm joints: {}".format(len(right_working_joints)))
    for joint in right_working_joints:
        print("  OK", joint)
    
    print("Working left arm joints: {}".format(len(left_working_joints)))
    for joint in left_working_joints:
        print("  OK", joint)
    
    print("\nNon-responsive joints:")
    all_right_joints = set(right_angles.keys())
    all_left_joints = set(left_angles.keys())
    
    failed_right = all_right_joints - set(right_working_joints)
    failed_left = all_left_joints - set(left_working_joints)
    
    if failed_right:
        print("Right arm issues:")
        for joint in failed_right:
            print("  FAIL", joint)
    
    if failed_left:
        print("Left arm issues:")
        for joint in failed_left:
            print("  FAIL", joint)
    
    if not failed_right and not failed_left:
        print("OK All joints responding normally!")
    
    # Recommendations
    print("\n=== RECOMMENDATIONS ===")
    
    if failed_right or failed_left:
        print("Issues detected. Try:")
        print("1. rosrun baxter_tools enable_robot.py -d && sleep 2 && rosrun baxter_tools enable_robot.py -e")
        print("2. rosrun baxter_tools tuck_arms.py -t && rosrun baxter_tools tuck_arms.py -u")
        print("3. Check for physical obstructions")
        print("4. Restart robot if problems persist")
    else:
        print("All joints working! Your demo should run with conservative movements.")
    
    print("\n=== DIAGNOSTICS COMPLETE ===")

if __name__ == "__main__":
    diagnose_baxter()