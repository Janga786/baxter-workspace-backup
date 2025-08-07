#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool, Empty
from baxter_core_msgs.msg import AssemblyState
import time

class BaxterCalibrationHelper:
    def __init__(self):
        rospy.init_node('baxter_calibration_helper', anonymous=True)
        
        # Publishers
        self.enable_pub = rospy.Publisher('/robot/set_super_enable', Bool, queue_size=1)
        self.disable_pub = rospy.Publisher('/robot/set_super_stop', Empty, queue_size=1)
        self.reset_pub = rospy.Publisher('/robot/set_super_reset', Empty, queue_size=1)
        
        # Subscribers
        self.robot_state = None
        self.assembly_state = None
        
        rospy.Subscriber('/robot/state', AssemblyState, self.robot_state_callback)
        
        # Wait for connections
        rospy.sleep(2.0)
        
    def robot_state_callback(self, msg):
        self.robot_state = msg
        
    def check_robot_status(self):
        """Check and display current robot status"""
        print("=" * 50)
        print("BAXTER ROBOT STATUS CHECK")
        print("=" * 50)
        
        # Check ROS connection
        try:
            rospy.get_master().getSystemState()
            print("Connected to ROS master at {}".format(rospy.get_param('/rosdistro', 'unknown')))
        except Exception as e:
            print("✗ Failed to connect to ROS master: {}".format(e))
            return False
            
        # Check available topics
        topics = rospy.get_published_topics()
        critical_topics = [
            '/robot/state',
            '/robot/set_super_enable',
            '/robot/limb/left/command_joint_position',
            '/robot/limb/right/command_joint_position'
        ]
        
        print("\nTopic availability:")
        all_topics_available = True
        for topic in critical_topics:
            available = any(t[0] == topic for t in topics)
            status = "✓" if available else "✗"
            print("  {} {}".format(status, topic))
            if not available:
                all_topics_available = False
                
        # Wait for robot state message
        print("\nWaiting for robot state...")
        timeout = rospy.Time.now() + rospy.Duration(5.0)
        while self.robot_state is None and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
            
        if self.robot_state is None:
            print("✗ Could not receive robot state message")
            print("  This usually means:")
            print("  - Robot software is not running properly")
            print("  - Robot is not powered on")
            print("  - Network connection issues")
            return False
            
        print("✓ Received robot state message")
        return True
        
    def display_robot_state(self):
        """Display detailed robot state information"""
        if self.robot_state is None:
            print("No robot state available")
            return
            
        print("\nROBOT STATE DETAILS:")
        print("-" * 30)
        print("Enabled: {}".format(self.robot_state.enabled))
        print("Stopped: {}".format(self.robot_state.stopped))
        print("Error: {}".format(self.robot_state.error))
        print("Estop Button: {}".format("PRESSED" if self.robot_state.estop_button else "RELEASED"))
        print("Estop Source: {}".format(self.robot_state.estop_source))
        
        # Check calibration status
        print("\nCALIBRATION STATUS:")
        print("-" * 30)
        
        # We need to check homing/calibration topics
        try:
            # Check if homing topics exist
            topics = rospy.get_published_topics()
            homing_topics = [t for t in topics if 'homing' in t[0].lower()]
            
            if homing_topics:
                print("Homing topics found:")
                for topic in homing_topics:
                    print("  - {}".format(topic[0]))
            else:
                print("No homing topics found")
                
        except Exception as e:
            print("Error checking calibration: {}".format(e))
            
    def diagnose_issues(self):
        """Diagnose common issues and provide solutions"""
        if self.robot_state is None:
            return
            
        print("\nDIAGNOSIS & SOLUTIONS:")
        print("=" * 50)
        
        issues_found = False
        
        # Check emergency stop
        if self.robot_state.estop_button:
            print("🚨 EMERGENCY STOP IS PRESSED!")
            print("   Solution: Release the red emergency stop button")
            print("   Location: Usually on the back of the robot or on a connected pendant")
            issues_found = True
            
        # Check if robot is stopped
        if self.robot_state.stopped:
            print("🚨 ROBOT IS STOPPED!")
            print("   This could be due to:")
            print("   - Emergency stop was pressed")
            print("   - Safety violation occurred") 
            print("   - Software error")
            print("   Solution: Try reset command")
            issues_found = True
            
        # Check if robot has errors
        if self.robot_state.error:
            print("🚨 ROBOT HAS ERROR STATE!")
            print("   Solution: Try reset command and check robot display")
            issues_found = True
            
        # Check if robot is not enabled
        if not self.robot_state.enabled:
            print("⚠️  ROBOT IS NOT ENABLED!")
            print("   Solution: Enable the robot using enable command")
            issues_found = True
            
        if not issues_found:
            print("✓ No obvious issues detected in robot state")
            print("  If robot still won't move, check:")
            print("  - Robot display for error messages")
            print("  - Joint calibration status")
            print("  - Physical obstructions")
            
    def attempt_recovery(self):
        """Attempt to recover the robot from error states"""
        print("\nATTEMPTING ROBOT RECOVERY:")
        print("=" * 40)
        
        if self.robot_state is None:
            print("Cannot attempt recovery - no robot state")
            return
            
        recovery_steps = []
        
        # Step 1: Reset if needed
        if self.robot_state.error or self.robot_state.stopped:
            recovery_steps.append("reset")
            
        # Step 2: Enable if needed  
        if not self.robot_state.enabled:
            recovery_steps.append("enable")
            
        if not recovery_steps:
            print("No recovery steps needed based on current state")
            return
            
        for step in recovery_steps:
            if step == "reset":
                print("1. Sending reset command...")
                reset_msg = Empty()
                for _ in range(3):
                    self.reset_pub.publish(reset_msg)
                    rospy.sleep(0.5)
                rospy.sleep(2.0)
                
            elif step == "enable":
                print("2. Sending enable command...")
                enable_msg = Bool()
                enable_msg.data = True
                for _ in range(5):
                    self.enable_pub.publish(enable_msg)
                    rospy.sleep(0.5)
                rospy.sleep(2.0)
                
        # Check state after recovery
        print("3. Checking state after recovery...")
        rospy.sleep(1.0)
        self.display_robot_state()
        
    def run_full_diagnostic(self):
        """Run complete diagnostic sequence"""
        print("Starting Baxter diagnostic sequence...")
        
        # Step 1: Basic status check
        if not self.check_robot_status():
            print("\n❌ CRITICAL: Basic connectivity failed")
            print("Please check:")
            print("- Robot is powered on")
            print("- Network connection to robot")
            print("- ROS master is running on robot")
            return False
            
        # Step 2: Display current state
        self.display_robot_state()
        
        # Step 3: Diagnose issues
        self.diagnose_issues()
        
        # Step 4: Attempt recovery if needed
        if (self.robot_state and 
            (self.robot_state.error or self.robot_state.stopped or not self.robot_state.enabled)):
            
            user_input = raw_input("\nAttempt automatic recovery? (y/n): ")
            if user_input.lower() in ['y', 'yes']:
                self.attempt_recovery()
                
        # Step 5: Final recommendations
        print("\nFINAL RECOMMENDATIONS:")
        print("=" * 40)
        
        if self.robot_state:
            if (not self.robot_state.estop_button and 
                not self.robot_state.error and 
                not self.robot_state.stopped and 
                self.robot_state.enabled):
                print("✅ Robot appears ready for movement!")
                print("   Try running the movement script again.")
            else:
                print("❌ Robot is not ready for movement")
                print("   Address the issues above first")
        else:
            print("❌ Could not determine robot state")
            
        return True

def main():
    try:
        helper = BaxterCalibrationHelper()
        helper.run_full_diagnostic()
        
    except rospy.ROSInterruptException:
        print("Program interrupted")
    except Exception as e:
        print("Error: {}".format(e))

if __name__ == '__main__':
    main()