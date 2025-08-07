#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Empty
import time

def manual_calibration_guide():
    """Provide step-by-step manual calibration guidance"""
    
    print("=" * 60)
    print("BAXTER MANUAL CALIBRATION & TROUBLESHOOTING GUIDE")
    print("=" * 60)
    
    print("\nSTEP 1: PHYSICAL INSPECTION")
    print("-" * 30)
    print("Before software checks, verify:")
    print("1. Robot is powered ON (green lights visible)")
    print("2. Emergency stop button is RELEASED (should pop out)")
    print("3. No physical obstructions around robot arms")
    print("4. Robot display screen is showing information (not blank)")
    print("5. Ethernet cable is connected to robot base")
    
    raw_input("\nPress Enter when physical inspection is complete...")
    
    print("\nSTEP 2: ROBOT DISPLAY CHECK")
    print("-" * 30)
    print("Look at the robot's head display screen.")
    print("It should show:")
    print("- Current robot status")
    print("- Joint positions")
    print("- Error messages (if any)")
    print("")
    print("Common error messages and solutions:")
    print("- 'Robot Disabled': Robot needs to be enabled")
    print("- 'Calibration Required': Joints need calibration")
    print("- 'E-Stop Active': Emergency stop is pressed")
    print("- 'Communication Error': Network/software issue")
    
    status = raw_input("\nWhat does the robot display show? (describe or type 'ok' if normal): ")
    
    if 'error' in status.lower() or 'calibrat' in status.lower():
        print("\n⚠️  ERROR DETECTED ON DISPLAY")
        print("The robot display is showing an error state.")
        print("This explains why movement commands are not working.")
        
    print("\nSTEP 3: MANUAL CALIBRATION PROCESS")
    print("-" * 30)
    print("If calibration is required, follow these steps:")
    print("")
    print("1. ENSURE SAFETY:")
    print("   - Clear area around robot")
    print("   - Be ready to press emergency stop if needed")
    print("   - Have someone supervise the process")
    print("")
    print("2. CALIBRATION BUTTON METHOD:")
    print("   - Look for calibration button on robot base or pendant")
    print("   - Press and hold the calibration button")
    print("   - Robot will move joints to find reference positions")
    print("   - This process takes 2-3 minutes")
    print("   - Do NOT interrupt this process")
    print("")
    print("3. SOFTWARE CALIBRATION METHOD:")
    print("   - Use the robot's display interface")
    print("   - Navigate to Settings > Calibration")
    print("   - Follow on-screen instructions")
    
    print("\nSTEP 4: NETWORK CONNECTIVITY TEST")
    print("-" * 30)
    
    # Test ROS connection
    rospy.init_node('manual_calibration_guide', anonymous=True)
    
    try:
        rospy.get_master().getSystemState()
        print("✓ ROS connection successful")
        
        # Test key topics
        topics = rospy.get_published_topics()
        key_topics = ['/robot/state', '/robot/set_super_enable']
        
        print("Topic availability:")
        for topic in key_topics:
            available = any(t[0] == topic for t in topics)
            status = "✓" if available else "✗"
            print("  {} {}".format(status, topic))
            
    except Exception as e:
        print("✗ ROS connection failed: {}".format(e))
        print("This indicates a network or software problem.")
        
    print("\nSTEP 5: ENABLE ROBOT")
    print("-" * 30)
    print("Attempting to enable robot...")
    
    try:
        enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
        rospy.sleep(2.0)
        
        enable_msg = Bool()
        enable_msg.data = True
        
        for i in range(5):
            enable_pub.publish(enable_msg)
            print("  Enable command {} sent".format(i+1))
            rospy.sleep(1.0)
            
        print("✓ Enable commands sent successfully")
        
    except Exception as e:
        print("✗ Failed to send enable commands: {}".format(e))
        
    print("\nSTEP 6: FINAL CHECKLIST")
    print("-" * 30)
    print("Before trying movement again, verify:")
    print("□ Emergency stop is released")
    print("□ Robot display shows 'Enabled' or normal status") 
    print("□ No error messages on display")
    print("□ Calibration completed (if it was required)")
    print("□ ROS topics are available")
    print("□ Network connection is stable")
    
    print("\nIf all items above are checked and robot still doesn't move:")
    print("1. Try power cycling the robot (turn off, wait 30 sec, turn on)")
    print("2. Check if robot firmware needs updating")
    print("3. Contact robot manufacturer support")
    
    print("\n" + "=" * 60)
    print("CALIBRATION GUIDE COMPLETE")
    print("You can now try running the movement script again.")
    print("=" * 60)

if __name__ == '__main__':
    try:
        manual_calibration_guide()
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print("Error: {}".format(e))