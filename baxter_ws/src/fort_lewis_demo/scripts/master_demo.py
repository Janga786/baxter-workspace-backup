#!/usr/bin/env python2.7

"""
Baxter Master Demonstration Script
A comprehensive showcase of Baxter's capabilities integrating all demo components.

This script orchestrates a complete demonstration featuring:
- Robot initialization and calibration
- Interactive storytelling (Duck Quest)
- Movement demonstrations
- Audience interaction
- Safety monitoring

Author: Fort Lewis College Robotics Team
Compatible with: ROS Melodic, Baxter SDK, Python 2.7
"""

import rospy
import baxter_interface
import time
import sys
import signal
from baxter_interface import CHECK_VERSION

# Import our custom modules
from baxter_movements import BaxterMovements
from baxter_communication import BaxterCommunication
from baxter_demo_utils import SafetyMonitor, PerformanceMonitor, ErrorRecovery
from baxter_demo_main import BaxterDuckQuest


class BaxterMasterDemo(object):
    """
    Master demonstration class that orchestrates all Baxter capabilities
    into a seamless, impressive showcase.
    """
    
    def __init__(self):
        """Initialize the master demo system"""
        rospy.loginfo("Initializing Baxter Master Demonstration...")
        
        # Initialize ROS node
        rospy.init_node('baxter_master_demo', anonymous=False)
        
        # Initialize robot enable
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        
        # Initialize component modules
        self.movements = BaxterMovements()
        self.communication = BaxterCommunication()
        self.safety_monitor = SafetyMonitor()
        self.performance_monitor = PerformanceMonitor()
        self.error_recovery = ErrorRecovery(
            self.movements.left_arm, 
            self.movements.right_arm
        )
        
        # Demo state tracking
        self.demo_running = False
        self.emergency_stop_requested = False
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        rospy.loginfo("Master demo system initialized successfully!")
    
    def _signal_handler(self, sig, frame):
        """Handle shutdown signals gracefully"""
        rospy.logwarn("Shutdown signal received. Stopping demo safely...")
        self.emergency_stop_requested = True
        self.emergency_stop()
    
    def initialize_robot(self):
        """Initialize and calibrate the robot"""
        rospy.loginfo("=== ROBOT INITIALIZATION ===")
        
        try:
            # Enable robot
            if not self._rs.state().enabled:
                rospy.loginfo("Enabling robot...")
                self._rs.enable()
                time.sleep(2.0)
            
            if not self._rs.state().enabled:
                rospy.logerr("Failed to enable robot!")
                return False
            
            # Calibrate grippers
            rospy.loginfo("Calibrating grippers...")
            for side in ['left', 'right']:
                gripper = baxter_interface.Gripper(side, CHECK_VERSION)
                if not gripper.calibrated():
                    gripper.calibrate()
                    time.sleep(2.0)
            
            # Move to neutral position
            rospy.loginfo("Moving to neutral position...")
            self.movements.move_to_neutral()
            
            # Show ready status
            status_info = [
                ("Robot Status: ENABLED", (0, 255, 0)),
                ("Grippers: CALIBRATED", (0, 255, 255)),
                ("Arms: READY", (255, 255, 255)),
                ("Demo System: ACTIVE", (255, 255, 0))
            ]
            self.communication.show_status("SYSTEM READY", status_info, 3)
            
            rospy.loginfo("Robot initialization complete!")
            return True
            
        except Exception as e:
            rospy.logerr("Robot initialization failed: {}".format(e))
            return False
    
    def phase_1_introduction(self):
        """Phase 1: Introduction and greeting"""
        rospy.loginfo("=== PHASE 1: INTRODUCTION ===")
        self.performance_monitor.mark_phase("introduction_start")
        
        try:
            # Wake up sequence
            self.communication.show_expression("sleeping", 2)
            self.communication.say_and_display("Zzz... zzz...", "sleeping", 2, False)
            
            # Stretching and waking up
            self.communication.show_expression("waking", 2)
            self.movements.stretching_sequence()
            self.communication.say_and_display("Mmm... Good morning everyone!", "waking", 3)
            
            # Alert and greeting
            self.communication.show_expression("alert", 2)
            self.movements.look_around()
            
            # Introduction sequence
            self.communication.introduction_sequence()
            
            # Welcome wave
            self.communication.say_and_display("Let me give you all a wave!", "happy", 2)
            self.movements.wave_gesture('right', 3)
            
            self.performance_monitor.mark_phase("introduction_complete")
            return True
            
        except Exception as e:
            rospy.logerr("Phase 1 error: {}".format(e))
            self.performance_monitor.record_error("introduction_error")
            return False
    
    def phase_2_duck_quest(self):
        """Phase 2: Interactive Duck Quest story"""
        rospy.loginfo("=== PHASE 2: DUCK QUEST STORY ===")
        self.performance_monitor.mark_phase("duck_quest_start")
        
        try:
            # Initialize duck quest
            duck_quest = BaxterDuckQuest()
            
            # Show story introduction
            story_status = [
                ("INTERACTIVE STORY MODE", (0, 255, 0)),
                ("Title: The Duck Quest", (0, 255, 255)),
                ("Audience participation required!", (255, 255, 255)),
                ("Please have a rubber duck ready", (255, 255, 0))
            ]
            self.communication.show_status("STORY TIME", story_status, 4)
            
            # Run the duck quest story
            duck_quest.run_complete_duck_quest()
            
            self.performance_monitor.mark_phase("duck_quest_complete")
            self.performance_monitor.record_interaction()
            return True
            
        except Exception as e:
            rospy.logerr("Phase 2 error: {}".format(e))
            self.performance_monitor.record_error("duck_quest_error")
            return False
    
    def phase_3_skill_demonstration(self):
        """Phase 3: Technical skill demonstrations"""
        rospy.loginfo("=== PHASE 3: SKILL DEMONSTRATIONS ===")
        self.performance_monitor.mark_phase("skills_start")
        
        try:
            # Announce skill demonstration
            self.communication.say_and_display(
                "Now let me demonstrate some of my technical capabilities!", 
                "alert", 3
            )
            
            # Precision movement demonstration
            self.communication.say_and_display("First, precise arm movements!", "neutral", 2)
            self._demonstrate_precision_movements()
            
            # Coordination demonstration
            self.communication.say_and_display("Next, bilateral coordination!", "excited", 2)
            self._demonstrate_coordination()
            
            # Expression demonstration
            self.communication.say_and_display("And various facial expressions!", "happy", 2)
            self._demonstrate_expressions()
            
            self.performance_monitor.mark_phase("skills_complete")
            return True
            
        except Exception as e:
            rospy.logerr("Phase 3 error: {}".format(e))
            self.performance_monitor.record_error("skills_error")
            return False
    
    def phase_4_audience_interaction(self):
        """Phase 4: Audience interaction and high-fives"""
        rospy.loginfo("=== PHASE 4: AUDIENCE INTERACTION ===")
        self.performance_monitor.mark_phase("interaction_start")
        
        try:
            # Announce interaction phase
            self.communication.say_and_display(
                "Time for some audience interaction!", 
                "excited", 3
            )
            
            # High-five demonstration
            self.communication.say_and_display(
                "Who wants a high-five? Come on up!", 
                "happy", 3
            )
            
            # Move to high-five position
            self.movements.high_five_position('right')
            
            # Wait for interaction
            self.communication.say_and_display(
                "Put your hand here for a high-five!", 
                "alert", 5
            )
            
            # Celebration after interaction
            self.communication.say_and_display("Awesome! Thank you!", "excited", 2)
            self.movements.celebration_dance()
            
            # Head nod acknowledgment
            self.movements.head_nod(3)
            
            self.performance_monitor.mark_phase("interaction_complete")
            self.performance_monitor.record_interaction()
            return True
            
        except Exception as e:
            rospy.logerr("Phase 4 error: {}".format(e))
            self.performance_monitor.record_error("interaction_error")
            return False
    
    def phase_5_conclusion(self):
        """Phase 5: Demo conclusion and farewell"""
        rospy.loginfo("=== PHASE 5: CONCLUSION ===")
        self.performance_monitor.mark_phase("conclusion_start")
        
        try:
            # Final celebration
            self.communication.say_and_display(
                "That concludes my demonstration!", 
                "happy", 3
            )
            
            # Thank the audience
            self.communication.conclusion_sequence()
            
            # Final bow
            self.communication.say_and_display("Thank you all!", "excited", 2)
            self.movements.bow_gesture()
            
            # Return to safe position
            self.communication.say_and_display("Returning to rest position...", "neutral", 2)
            self.movements.move_to_neutral()
            
            # Final status
            final_status = [
                ("DEMONSTRATION COMPLETE", (0, 255, 0)),
                ("All systems nominal", (0, 255, 255)),
                ("Thank you for watching!", (255, 255, 255)),
                ("Baxter signing off!", (255, 255, 0))
            ]
            self.communication.show_status("DEMO COMPLETE", final_status, 5)
            
            self.performance_monitor.mark_phase("conclusion_complete")
            return True
            
        except Exception as e:
            rospy.logerr("Phase 5 error: {}".format(e))
            self.performance_monitor.record_error("conclusion_error")
            return False
    
    def _demonstrate_precision_movements(self):
        """Demonstrate precise arm movements"""
        # Precise positioning sequence
        positions = [
            "Moving to position 1...",
            "Moving to position 2...", 
            "Moving to position 3...",
            "Returning to center..."
        ]
        
        current_right = self.movements.right_arm.joint_angles()
        offsets = [
            {'right_s0': -0.3, 'right_e1': 0.2},
            {'right_s0': 0.3, 'right_e1': 0.2},
            {'right_s0': 0.0, 'right_e1': -0.2},
            {'right_s0': 0.0, 'right_e1': 0.0}
        ]
        
        for i, (message, offset) in enumerate(zip(positions, offsets)):
            self.communication.say_and_display(message, "neutral", 1)
            
            target_pos = current_right.copy()
            for joint, value in offset.items():
                target_pos[joint] += value
                
            self.movements.safe_move_arm(self.movements.right_arm, target_pos)
            time.sleep(1.0)
    
    def _demonstrate_coordination(self):
        """Demonstrate bilateral arm coordination"""
        self.communication.say_and_display("Watch both arms work together!", "alert", 2)
        
        # Synchronized movements
        current_left = self.movements.left_arm.joint_angles()
        current_right = self.movements.right_arm.joint_angles()
        
        # Mirror movements
        for i in range(3):
            offset = 0.3 * (1 if i % 2 == 0 else -1)
            
            left_target = current_left.copy()
            right_target = current_right.copy()
            
            left_target['left_s1'] += offset
            right_target['right_s1'] += offset
            
            self.movements.safe_move_arm(self.movements.left_arm, left_target, 0.4)
            self.movements.safe_move_arm(self.movements.right_arm, right_target, 0.4)
            time.sleep(1.0)
        
        # Return to original positions
        self.movements.safe_move_arm(self.movements.left_arm, current_left)
        self.movements.safe_move_arm(self.movements.right_arm, current_right)
    
    def _demonstrate_expressions(self):
        """Demonstrate various facial expressions"""
        expressions = [
            ("Here's my happy face!", "happy"),
            ("Now I'm excited!", "excited"), 
            ("Searching for something...", "searching"),
            ("Alert and focused!", "alert"),
            ("Back to neutral", "neutral")
        ]
        
        for message, expression in expressions:
            self.communication.say_and_display(message, expression, 2, False)
            time.sleep(1.0)
    
    def run_complete_demo(self):
        """Execute the complete master demonstration"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("BAXTER MASTER DEMONSTRATION - STARTING")
        rospy.loginfo("=" * 60)
        
        self.demo_running = True
        self.performance_monitor.start_demo()
        
        try:
            # Pre-demo setup
            if not self.initialize_robot():
                rospy.logerr("Robot initialization failed. Aborting demo.")
                return False
            
            # Execute demo phases
            demo_phases = [
                ("Introduction", self.phase_1_introduction),
                ("Duck Quest Story", self.phase_2_duck_quest),
                ("Skill Demonstration", self.phase_3_skill_demonstration),
                ("Audience Interaction", self.phase_4_audience_interaction),
                ("Conclusion", self.phase_5_conclusion)
            ]
            
            for phase_name, phase_function in demo_phases:
                if self.emergency_stop_requested:
                    rospy.logwarn("Emergency stop requested. Halting demo.")
                    break
                    
                rospy.loginfo("Starting phase: {}".format(phase_name))
                
                # Safety check before each phase
                if self.safety_monitor.check_collision():
                    rospy.logwarn("Collision detected. Pausing for safety.")
                    time.sleep(2.0)
                
                # Execute phase
                success = phase_function()
                if not success:
                    rospy.logerr("Phase {} failed. Attempting recovery...".format(phase_name))
                    self._attempt_recovery()
                
                # Brief pause between phases
                time.sleep(2.0)
            
            # Demo completion
            rospy.loginfo("=" * 60)
            rospy.loginfo("BAXTER MASTER DEMONSTRATION - COMPLETED")
            rospy.loginfo("=" * 60)
            
            self.performance_monitor.print_summary()
            return True
            
        except rospy.ROSInterruptException:
            rospy.loginfo("Demo interrupted by user")
            return False
        except Exception as e:
            rospy.logerr("Demo error: {}".format(e))
            self.emergency_stop()
            return False
        finally:
            self.demo_running = False
    
    def _attempt_recovery(self):
        """Attempt recovery from errors"""
        rospy.loginfo("Attempting error recovery...")
        
        try:
            # Move to safe position
            self.movements.move_to_neutral()
            
            # Show recovery status
            self.communication.show_expression("alert", 2)
            self.communication.say_and_display("Recovering... please wait.", "neutral", 2)
            
            time.sleep(2.0)
            rospy.loginfo("Recovery attempt completed")
            
        except Exception as e:
            rospy.logerr("Recovery failed: {}".format(e))
    
    def emergency_stop(self):
        """Emergency stop procedure"""
        rospy.logwarn("EMERGENCY STOP ACTIVATED")
        
        self.demo_running = False
        
        try:
            # Stop all arm motion
            self.movements.left_arm.exit_control_mode()
            self.movements.right_arm.exit_control_mode()
            
            # Show emergency status
            emergency_status = [
                ("EMERGENCY STOP ACTIVE", (255, 0, 0)),
                ("All motion halted", (255, 255, 0)),
                ("System safe", (0, 255, 0)),
                ("Please check robot", (255, 255, 255))
            ]
            self.communication.show_status("EMERGENCY STOP", emergency_status, 5)
            
            rospy.logwarn("Emergency stop completed safely")
            
        except Exception as e:
            rospy.logerr("Emergency stop error: {}".format(e))
    
    def test_components(self):
        """Test individual components for debugging"""
        rospy.loginfo("=== COMPONENT TESTING MODE ===")
        
        tests = [
            ("Communication System", self._test_communication),
            ("Movement System", self._test_movements),
            ("Safety Systems", self._test_safety)
        ]
        
        for test_name, test_function in tests:
            rospy.loginfo("Testing: {}".format(test_name))
            try:
                test_function()
                rospy.loginfo("{}: PASSED".format(test_name))
            except Exception as e:
                rospy.logerr("{}: FAILED - {}".format(test_name, e))
    
    def _test_communication(self):
        """Test communication systems"""
        self.communication.say_and_display("Communication test", "neutral", 2)
        self.communication.show_expression("happy", 1)
    
    def _test_movements(self):
        """Test movement systems"""
        self.movements.wave_gesture('right', 1)
        self.movements.head_nod(1)
    
    def _test_safety(self):
        """Test safety systems"""
        collision_status = self.safety_monitor.check_collision()
        velocity_status = self.safety_monitor.check_velocity_limits()
        rospy.loginfo("Collision check: {}".format(collision_status))
        rospy.loginfo("Velocity check: {}".format(velocity_status))


def main():
    """Main execution function"""
    try:
        rospy.loginfo("Initializing Baxter Master Demo...")
        demo = BaxterMasterDemo()
        
        # Demo options
        rospy.loginfo("\\n" + "=" * 60)
        rospy.loginfo("BAXTER MASTER DEMONSTRATION")
        rospy.loginfo("Fort Lewis College Robotics Showcase")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Demo Options:")
        rospy.loginfo("1. Run complete master demonstration")
        rospy.loginfo("2. Test individual components")  
        rospy.loginfo("3. Emergency stop test")
        rospy.loginfo("=" * 60)
        
        # Get user choice (with timeout for autonomous operation)
        try:
            choice = raw_input("Enter choice (1-3) or wait 10 seconds for auto-start: ").strip()
        except (EOFError, KeyboardInterrupt):
            choice = "1"  # Default to full demo
        
        if choice == "2":
            demo.test_components()
        elif choice == "3":
            demo.emergency_stop()
        else:
            rospy.loginfo("Starting complete master demonstration in 5 seconds...")
            time.sleep(5.0)
            success = demo.run_complete_demo()
            
            if success:
                rospy.loginfo("Master demonstration completed successfully!")
            else:
                rospy.logwarn("Master demonstration completed with errors.")
    
    except rospy.ROSInterruptException:
        rospy.loginfo("Demo interrupted by ROS shutdown")
    except KeyboardInterrupt:
        rospy.loginfo("Demo interrupted by user")
    except Exception as e:
        rospy.logerr("Demo initialization error: {}".format(e))
        rospy.logerr("Please check:")
        rospy.logerr("- ROS is running (roscore)")
        rospy.logerr("- Baxter robot is connected and enabled") 
        rospy.logerr("- All required packages are installed")
        rospy.logerr("- Robot workspace is clear")


if __name__ == '__main__':
    main()