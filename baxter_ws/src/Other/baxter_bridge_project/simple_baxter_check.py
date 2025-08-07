#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Empty
from baxter_core_msgs.msg import AssemblyState
import time

def simple_baxter_check():
    """Simple Baxter status check and recovery"""
    
    rospy.init_node('simple_baxter_check', anonymous=True)
    
    print("BAXTER ROBOT DIAGNOSTIC")
    print("=" * 40)
    
    # Publishers
    enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
    reset_pub = rospy.Publisher('/robot/set_super_reset', Empty, queue_size=1)
    
    # Check ROS connection
    try:
        rospy.get_master().getSystemState()
        print("OK: Connected to ROS master")
    except Exception as e:
        print("ERROR: Cannot connect to ROS master - {}".format(e))
        return
        
    # Check topics
    topics = rospy.get_published_topics()
    critical_topics = [
        '/robot/state',
        '/robot/set_super_enable', 
        '/robot/limb/left/command_joint_position'
    ]
    
    print("\nTopic Check:")
    for topic in critical_topics:
        available = any(t[0] == topic for t in topics)
        status = "OK" if available else "MISSING"
        print("  {}: {}".format(topic, status))
        
    # Wait for robot state
    print("\nWaiting for robot state...")
    robot_state = None
    
    def state_callback(msg):
        global robot_state
        robot_state = msg
        
    rospy.Subscriber('/robot/state', AssemblyState, state_callback)
    
    # Wait up to 5 seconds for state
    timeout = time.time() + 5
    while robot_state is None and time.time() < timeout:
        rospy.sleep(0.1)
        
    if robot_state is None:
        print("ERROR: No robot state received")
        print("Possible causes:")
        print("- Robot software not running")
        print("- Robot not powered on")
        print("- Network issues")
        return
        
    print("OK: Robot state received")
    
    # Display robot state
    print("\nRobot Status:")
    print("  Enabled: {}".format(robot_state.enabled))
    print("  Stopped: {}".format(robot_state.stopped))
    print("  Error: {}".format(robot_state.error))
    print("  E-Stop: {}".format("PRESSED" if robot_state.estop_button else "RELEASED"))
    
    # Diagnose issues
    issues = []
    if robot_state.estop_button:
        issues.append("Emergency stop is pressed - release the red button")
    if robot_state.stopped:
        issues.append("Robot is stopped - needs reset")
    if robot_state.error:
        issues.append("Robot has error state - needs reset")
    if not robot_state.enabled:
        issues.append("Robot is not enabled - needs enabling")
        
    if issues:
        print("\nISSUES FOUND:")
        for i, issue in enumerate(issues, 1):
            print("  {}: {}".format(i, issue))
            
        print("\nAttempting automatic recovery...")
        
        # Reset if needed
        if robot_state.stopped or robot_state.error:
            print("Sending reset command...")
            reset_msg = Empty()
            for _ in range(3):
                reset_pub.publish(reset_msg)
                rospy.sleep(0.5)
            rospy.sleep(2)
            
        # Enable if needed
        if not robot_state.enabled:
            print("Sending enable command...")
            enable_msg = Bool()
            enable_msg.data = True
            for _ in range(5):
                enable_pub.publish(enable_msg)
                rospy.sleep(0.5)
                
        print("Recovery commands sent.")
        print("Wait 10 seconds and try movement again.")
        
    else:
        print("\nOK: No software issues detected")
        print("Robot should be ready for movement commands")
        
    print("\nMANUAL CHECKS NEEDED:")
    print("1. Check robot display for error messages")
    print("2. Verify emergency stop button is released") 
    print("3. Ensure robot completed calibration")
    print("4. Check for physical obstructions")
    
    print("\nIf robot still won't move after this check:")
    print("- Robot may need manual calibration")
    print("- Check robot display for specific errors")
    print("- Try power cycling the robot")

if __name__ == '__main__':
    try:
        simple_baxter_check()
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except Exception as e:
        print("Error: {}".format(e))