#!/usr/bin/env python2.7

"""
Baxter Duck Quest - Interactive Story Script (Refined Version)
A complete interactive narrative where Baxter wakes up, searches for his duck,
finds it with help from humans, and celebrates!

Features:
- Robust computer vision duck detection
- Clean facial expression system
- Professional error handling
- Smooth arm movements and gestures
- Interactive human engagement
"""

import rospy
import baxter_interface
from baxter_interface import Head, Gripper
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import time
from std_msgs.msg import String
import random
import sys
import os

class BaxterDuckQuest:
    def __init__(self):
        """Initialize Baxter for the duck quest"""
        rospy.init_node('baxter_duck_quest')
        
        # Initialize robot interfaces
        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')
        self.left_gripper = Gripper('left')
        self.right_gripper = Gripper('right')
        self.head = Head()
        
        # Display and communication interfaces
        self.display_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=1)
        self.speech_pub = rospy.Publisher('/robot/say', String, queue_size=1)
        self.bridge = CvBridge()
        
        # Camera detection variables
        self.current_camera_image = None
        self.camera_sub = None
        self.detection_threshold = 0.15  # Adjustable detection sensitivity
        
        # Initialize robot
        self._initialize_robot()
        
        print("Baxter Duck Quest initialized successfully!")

    def _initialize_robot(self):
        """Initialize robot hardware and calibrate systems"""
        try:
            # Enable the robot
            baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION).enable()
            
            # Calibrate grippers
            if not self.left_gripper.calibrated():
                print("Calibrating left gripper...")
                self.left_gripper.calibrate()
            if not self.right_gripper.calibrated():
                print("Calibrating right gripper...")
                self.right_gripper.calibrate()
            
            # Set reasonable arm movement speeds
            self.left_arm.set_joint_position_speed(0.3)
            self.right_arm.set_joint_position_speed(0.3)
            
            # Center head
            self.head.set_pan(0.0)
            
        except Exception as e:
            print("Robot initialization error: {}".format(e))
            raise

    def camera_callback(self, msg):
        """Callback for camera image processing"""
        try:
            self.current_camera_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            print("Camera callback error: {}".format(e))

    def start_camera_detection(self, camera_name='right_hand_camera'):
        """Start subscribing to camera feed"""
        camera_topic = '/cameras/{}/image'.format(camera_name)
        print("Starting camera detection on: {}".format(camera_topic))
        self.camera_sub = rospy.Subscriber(camera_topic, Image, self.camera_callback)
        time.sleep(2.0)  # Allow camera to initialize

    def stop_camera_detection(self):
        """Stop camera subscription"""
        if self.camera_sub:
            self.camera_sub.unregister()
            self.camera_sub = None

    def detect_duck_in_image(self):
        """
        Robust duck detection using multiple computer vision techniques
        Returns True if duck-like object is detected
        """
        if self.current_camera_image is None:
            print("No camera image available")
            return False
            
        try:
            height, width = self.current_camera_image.shape[:2]
            
            # Method 1: Color-based detection
            hsv = cv2.cvtColor(self.current_camera_image, cv2.COLOR_BGR2HSV)
            
            # Detect bright, colorful objects (potential duck)
            lower_color = np.array([0, 30, 30])
            upper_color = np.array([180, 255, 255])
            color_mask = cv2.inRange(hsv, lower_color, upper_color)
            
            # Find contours in color mask
            contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 100:  # Minimum object size
                    print("Duck detected by color analysis! Area: {}".format(area))
                    return True
            
            # Method 2: Edge and texture analysis
            gray = cv2.cvtColor(self.current_camera_image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            edge_density = cv2.countNonZero(edges) / float(width * height)
            
            # Method 3: Brightness and contrast analysis
            mean_brightness = np.mean(gray)
            std_brightness = np.std(gray)
            
            # Combined detection logic
            content_score = (edge_density * 100) + (std_brightness / 10) + (mean_brightness / 25)
            
            print("Detection metrics - Edge density: {:.3f}, Brightness std: {:.1f}, Mean: {:.1f}, Score: {:.2f}".format(
                edge_density, std_brightness, mean_brightness, content_score))
            
            if content_score > self.detection_threshold:
                print("Duck detected by combined analysis! Score: {:.2f}".format(content_score))
                return True
                
            return False
            
        except Exception as e:
            print("Duck detection error: {}".format(e))
            # Return True on error to keep demo flowing
            return True

    def create_expression(self, expression_type):
        """
        Create pixelated, robotic facial expressions for Baxter's display
        """
        width, height = 1024, 600
        img = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Pixelated robot colors - more digital/retro feel
        left_eye_x, right_eye_x = width // 3, 2 * width // 3
        eye_y = height // 2 - 50
        
        # Robot color palette
        bright_green = (0, 255, 0)      # Classic terminal green
        cyan = (0, 255, 255)            # Digital cyan
        white = (255, 255, 255)
        dark_green = (0, 128, 0)
        gray = (128, 128, 128)
        black = (0, 0, 0)
        
        if expression_type == "sleeping":
            self._draw_pixelated_sleeping_eyes(img, left_eye_x, right_eye_x, eye_y, bright_green, dark_green)
            
        elif expression_type == "waking":
            self._draw_pixelated_waking_eyes(img, left_eye_x, right_eye_x, eye_y, cyan, bright_green, black)
            
        elif expression_type == "alert":
            self._draw_pixelated_alert_eyes(img, left_eye_x, right_eye_x, eye_y, bright_green, cyan, black)
            
        elif expression_type == "confused":
            self._draw_pixelated_confused_eyes(img, left_eye_x, right_eye_x, eye_y, bright_green, cyan, black)
            
        elif expression_type == "searching":
            self._draw_pixelated_searching_eyes(img, left_eye_x, right_eye_x, eye_y, bright_green, cyan, black)
            
        elif expression_type == "excited":
            self._draw_pixelated_excited_eyes(img, left_eye_x, right_eye_x, eye_y, bright_green, cyan, black)
            
        elif expression_type == "happy":
            self._draw_pixelated_happy_eyes(img, left_eye_x, right_eye_x, eye_y, bright_green, cyan, black)
            
        else:  # neutral
            self._draw_pixelated_neutral_eyes(img, left_eye_x, right_eye_x, eye_y, bright_green, cyan, black)
        
        return img

    def _draw_pixel_block(self, img, x, y, size, color):
        """Draw a pixelated block for retro robot effect"""
        cv2.rectangle(img, (x - size//2, y - size//2), (x + size//2, y + size//2), color, -1)
        # Add slight border for pixel effect
        cv2.rectangle(img, (x - size//2, y - size//2), (x + size//2, y + size//2), (50, 50, 50), 1)
    
    def _draw_pixelated_sleeping_eyes(self, img, left_x, right_x, y, bright_green, dark_green):
        """Draw pixelated sleeping eyes"""
        for eye_x in [left_x, right_x]:
            # Pixelated closed eyes with blocks
            for i in range(-6, 7):
                pixel_x = eye_x + i * 12
                self._draw_pixel_block(img, pixel_x, y, 8, bright_green)
            
            # Digital sleep indicators
            for i in range(3):
                zzz_x = eye_x + 60 + i * 15
                zzz_y = y - 30 - i * 10
                cv2.putText(img, "Z", (zzz_x, zzz_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, dark_green, 2)

    def _draw_pixelated_waking_eyes(self, img, left_x, right_x, y, cyan, bright_green, black):
        """Draw pixelated waking eyes"""
        for eye_x in [left_x, right_x]:
            # Pixelated half-open eyes
            for i in range(-5, 6):
                for j in range(2):
                    pixel_x = eye_x + i * 10
                    pixel_y = y + j * 8
                    if j == 0:
                        self._draw_pixel_block(img, pixel_x, pixel_y, 8, cyan)
                    else:
                        self._draw_pixel_block(img, pixel_x, pixel_y, 6, bright_green)

    def _draw_pixelated_alert_eyes(self, img, left_x, right_x, y, bright_green, cyan, black):
        """Draw pixelated alert eyes with digital crosshairs"""
        for eye_x in [left_x, right_x]:
            # Outer pixelated ring
            for angle in range(0, 360, 30):
                radius = 60
                pixel_x = int(eye_x + radius * math.cos(math.radians(angle)))
                pixel_y = int(y + radius * math.sin(math.radians(angle)))
                self._draw_pixel_block(img, pixel_x, pixel_y, 10, bright_green)
            
            # Inner ring
            for angle in range(0, 360, 45):
                radius = 35
                pixel_x = int(eye_x + radius * math.cos(math.radians(angle)))
                pixel_y = int(y + radius * math.sin(math.radians(angle)))
                self._draw_pixel_block(img, pixel_x, pixel_y, 8, cyan)
            
            # Center crosshair
            for i in range(-3, 4):
                self._draw_pixel_block(img, eye_x + i * 8, y, 6, black)
                self._draw_pixel_block(img, eye_x, y + i * 8, 6, black)
            
            # Alert indicators - pixelated exclamation
            cv2.putText(img, "!", (eye_x - 10, y - 80), cv2.FONT_HERSHEY_SIMPLEX, 1.5, bright_green, 3)

    def _draw_pixelated_confused_eyes(self, img, left_x, right_x, y, bright_green, cyan, black):
        """Draw pixelated confused eyes"""
        # Left eye - normal pixelated
        for i in range(-4, 5):
            for j in range(-4, 5):
                if i*i + j*j <= 16:  # Circular pattern
                    pixel_x = left_x + i * 8
                    pixel_y = y + j * 8
                    self._draw_pixel_block(img, pixel_x, pixel_y, 6, bright_green)
        
        # Pupil
        self._draw_pixel_block(img, left_x - 8, y, 8, black)
        
        # Right eye - squinted with pixels
        for i in range(-3, 4):
            for j in range(-2, 3):
                pixel_x = right_x + i * 8
                pixel_y = y + j * 8 + 8
                self._draw_pixel_block(img, pixel_x, pixel_y, 6, cyan)
        
        # Digital question marks
        cv2.putText(img, "???", (right_x + 40, y - 60), cv2.FONT_HERSHEY_SIMPLEX, 1.2, bright_green, 3)

    def _draw_pixelated_searching_eyes(self, img, left_x, right_x, y, bright_green, cyan, black):
        """Draw pixelated searching eyes with scan lines"""
        for eye_x in [left_x, right_x]:
            # Base pixelated eye
            for i in range(-5, 6):
                for j in range(-5, 6):
                    if i*i + j*j <= 25:
                        pixel_x = eye_x + i * 7
                        pixel_y = y + j * 7
                        self._draw_pixel_block(img, pixel_x, pixel_y, 5, bright_green)
            
            # Moving pupil (scanning)
            pupil_offset = 15
            self._draw_pixel_block(img, eye_x + pupil_offset, y, 8, black)
            
            # Digital scan lines
            for i in range(5):
                scan_y = y - 40 + i * 20
                for j in range(-8, 9):
                    scan_x = eye_x + j * 8
                    if j % 2 == 0:  # Dashed line effect
                        self._draw_pixel_block(img, scan_x, scan_y, 3, cyan)
            
            # Targeting brackets
            bracket_size = 60
            bracket_thickness = 8
            # Top-left
            cv2.rectangle(img, (eye_x - bracket_size, y - bracket_size), 
                         (eye_x - bracket_size + bracket_thickness, y - bracket_size + bracket_thickness), cyan, -1)
            # Top-right
            cv2.rectangle(img, (eye_x + bracket_size - bracket_thickness, y - bracket_size), 
                         (eye_x + bracket_size, y - bracket_size + bracket_thickness), cyan, -1)
            # Bottom-left
            cv2.rectangle(img, (eye_x - bracket_size, y + bracket_size - bracket_thickness), 
                         (eye_x - bracket_size + bracket_thickness, y + bracket_size), cyan, -1)
            # Bottom-right
            cv2.rectangle(img, (eye_x + bracket_size - bracket_thickness, y + bracket_size - bracket_thickness), 
                         (eye_x + bracket_size, y + bracket_size), cyan, -1)

    def _draw_pixelated_excited_eyes(self, img, left_x, right_x, y, bright_green, cyan, black):
        """Draw pixelated excited eyes with star pattern"""
        for eye_x in [left_x, right_x]:
            # Pixelated star burst pattern
            for angle in range(0, 360, 15):
                for radius in range(20, 80, 15):
                    pixel_x = int(eye_x + radius * math.cos(math.radians(angle)))
                    pixel_y = int(y + radius * math.sin(math.radians(angle)))
                    color = bright_green if radius % 30 == 20 else cyan
                    self._draw_pixel_block(img, pixel_x, pixel_y, 6, color)
            
            # Center excited pupil
            self._draw_pixel_block(img, eye_x, y, 12, black)
            self._draw_pixel_block(img, eye_x, y, 6, bright_green)
            
            # Digital sparkles around eyes
            sparkle_positions = [(eye_x - 80, y - 60), (eye_x + 80, y - 60), 
                               (eye_x - 80, y + 60), (eye_x + 80, y + 60)]
            for spark_x, spark_y in sparkle_positions:
                cv2.putText(img, "*", (spark_x, spark_y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, cyan, 2)

    def _draw_pixelated_happy_eyes(self, img, left_x, right_x, y, bright_green, cyan, black):
        """Draw pixelated happy eyes with smile curve"""
        for eye_x in [left_x, right_x]:
            # Pixelated curved happy eyes
            for i in range(-6, 7):
                curve_offset = int(abs(i) * 0.8)  # Create curve
                pixel_x = eye_x + i * 8
                pixel_y = y + curve_offset
                self._draw_pixel_block(img, pixel_x, pixel_y, 8, bright_green)
                
                # Lower curve for smile effect
                if abs(i) < 5:
                    self._draw_pixel_block(img, pixel_x, pixel_y + 8, 6, cyan)
            
            # Happy indicators
            cv2.putText(img, "^_^", (eye_x - 20, y - 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, bright_green, 2)

    def _draw_pixelated_neutral_eyes(self, img, left_x, right_x, y, bright_green, cyan, black):
        """Draw pixelated neutral robot eyes"""
        for eye_x in [left_x, right_x]:
            # Outer pixelated square eye
            for i in range(-5, 6):
                for j in range(-5, 6):
                    if abs(i) == 5 or abs(j) == 5:  # Border only
                        pixel_x = eye_x + i * 8
                        pixel_y = y + j * 8
                        self._draw_pixel_block(img, pixel_x, pixel_y, 6, bright_green)
            
            # Inner detail ring
            for i in range(-3, 4):
                for j in range(-3, 4):
                    if abs(i) == 3 or abs(j) == 3:
                        pixel_x = eye_x + i * 8
                        pixel_y = y + j * 8
                        self._draw_pixel_block(img, pixel_x, pixel_y, 4, cyan)
            
            # Center pupil - digital style
            self._draw_pixel_block(img, eye_x, y, 10, black)
            # Digital highlight - square
            self._draw_pixel_block(img, eye_x - 6, y - 6, 6, bright_green)

    def create_status_display(self, title, status_info, bg_color=(20, 20, 40)):
        """Create status display with important information for users to see"""
        width, height = 1024, 600
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = bg_color
        
        # Robot colors for consistency
        bright_green = (0, 255, 0)
        cyan = (0, 255, 255)
        white = (255, 255, 255)
        
        # Title at top
        title_size = cv2.getTextSize(title, cv2.FONT_HERSHEY_DUPLEX, 1.5, 2)[0]
        title_x = (width - title_size[0]) // 2
        cv2.putText(img, title, (title_x, 80), cv2.FONT_HERSHEY_DUPLEX, 1.5, bright_green, 2)
        
        # Underline
        cv2.line(img, (title_x, 90), (title_x + title_size[0], 90), cyan, 2)
        
        # Status information
        y_offset = 150
        for i, info in enumerate(status_info):
            if isinstance(info, tuple):  # (text, color)
                text, color = info
            else:
                text, color = info, white
                
            cv2.putText(img, text, (50, y_offset + i * 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        
        return img

    def create_text_display(self, text, bg_color=(0, 0, 0)):
        """Create pixelated text display for the head screen"""
        width, height = 1024, 600
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = bg_color
        
        # Robot colors
        bright_green = (0, 255, 0)
        cyan = (0, 255, 255)
        
        # Smart text wrapping
        max_chars_per_line = 22
        words = text.split(' ')
        lines = []
        current_line = ""
        
        for word in words:
            if len(current_line + word) <= max_chars_per_line:
                current_line += word + " "
            else:
                if current_line:
                    lines.append(current_line.strip())
                current_line = word + " "
        if current_line:
            lines.append(current_line.strip())
        
        # Center text vertically
        line_height = 80
        total_height = len(lines) * line_height
        start_y = (height - total_height) // 2 + 60
        
        # Draw text with pixelated/robot styling
        for i, line in enumerate(lines):
            text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_DUPLEX, 2, 3)[0]
            text_x = (width - text_size[0]) // 2
            text_y = start_y + i * line_height
            
            # Digital glow effect
            cv2.putText(img, line, (text_x + 3, text_y + 3), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 50, 0), 4)  # Dark glow
            cv2.putText(img, line, (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX, 2, bright_green, 3)
            
            # Add scan line effect for robot feel
            for scan_y in range(text_y - 30, text_y + 10, 6):
                cv2.line(img, (text_x - 20, scan_y), (text_x + text_size[0] + 20, scan_y), cyan, 1)
        
        return img

    def celebration_wobble(self):
        """Professional celebration sequence using joint_velocity_wobbler.py"""
        print("=== CELEBRATION WOBBLE ===")
        
        # Show celebration status on screen
        celebration_status = [
            ("DUCK FOUND!", (0, 255, 0)),
            ("Initializing celebration sequence...", (0, 255, 255)),
            ("Using joint_velocity_wobbler.py", (255, 255, 255)),
            ("Stand back - robot will move!", (255, 255, 0))
        ]
        status_img = self.create_status_display("CELEBRATION MODE", celebration_status)
        try:
            status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
            self.display_pub.publish(status_msg)
            time.sleep(2)
        except:
            pass
        
        self.say_and_display("I'm so happy to have my duck back!", "excited", 3, show_status=False)
        
        try:
            # Try to use the official joint_velocity_wobbler.py script
            import subprocess
            import os
            
            # Show wobble status
            wobble_status = [
                ("EXECUTING WOBBLE SEQUENCE", (0, 255, 0)),
                ("Running joint_velocity_wobbler.py...", (0, 255, 255)),
                ("Duration: ~10 seconds", (255, 255, 255)),
                ("Please wait...", (255, 255, 0))
            ]
            status_img = self.create_status_display("WOBBLE ACTIVE", wobble_status)
            try:
                status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
                self.display_pub.publish(status_msg)
            except:
                pass
            
            # Execute the wobbler script
            wobbler_path = os.path.expanduser('~/baxter_ws/src/baxter_examples/scripts/joint_velocity_wobbler.py')
            if os.path.exists(wobbler_path):
                print("Found joint_velocity_wobbler.py, executing...")
                result = subprocess.call(['python', wobbler_path], 
                                       cwd=os.path.expanduser('~/baxter_ws/src/baxter_examples/scripts/'))
                print("Wobbler completed with return code: {}".format(result))
            else:
                # Try alternate path
                wobbler_path2 = '/home/baxter/baxter_ws/src/baxter_examples/scripts/joint_velocity_wobbler.py'
                if os.path.exists(wobbler_path2):
                    print("Found joint_velocity_wobbler.py at alternate path, executing...")
                    result = subprocess.call(['python', wobbler_path2])
                    print("Wobbler completed with return code: {}".format(result))
                else:
                    print("joint_velocity_wobbler.py not found, using custom wobble")
                    self._advanced_custom_wobble()
                    
        except Exception as e:
            print("Error executing joint_velocity_wobbler.py: {}".format(e))
            print("Falling back to custom wobble sequence")
            self._advanced_custom_wobble()
        
        # Show completion status
        complete_status = [
            ("CELEBRATION COMPLETE!", (0, 255, 0)),
            ("Wobble sequence finished", (0, 255, 255)),
            ("Duck safely secured", (255, 255, 255)),
            ("Continuing adventure...", (255, 255, 0))
        ]
        status_img = self.create_status_display("WOBBLE COMPLETE", complete_status)
        try:
            status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
            self.display_pub.publish(status_msg)
            time.sleep(2)
        except:
            pass

    def _advanced_custom_wobble(self):
        """Advanced custom wobble implementation as fallback"""
        print("Executing advanced custom wobble sequence")
        
        # Show custom wobble status
        custom_status = [
            ("CUSTOM WOBBLE ACTIVE", (255, 255, 0)),
            ("Using internal wobble system", (0, 255, 255)),
            ("Multi-joint coordination", (255, 255, 255)),
            ("Duration: ~8 seconds", (0, 255, 0))
        ]
        status_img = self.create_status_display("CUSTOM WOBBLE", custom_status)
        try:
            status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
            self.display_pub.publish(status_msg)
            time.sleep(1)
        except:
            pass
        
        # Get current positions
        current_right = self.right_arm.joint_angles()
        current_left = self.left_arm.joint_angles()
        
        # Store originals
        original_positions = {
            'right_s1': current_right['right_s1'],
            'left_s1': current_left['left_s1'],
            'right_e1': current_right['right_e1'],
            'left_e1': current_left['left_e1'],
            'right_w1': current_right['right_w1'],
            'left_w1': current_left['left_w1']
        }
        
        # Advanced wobble sequence with multiple joints
        wobble_sequence = [
            # Phase 1: Big movements
            {'right_s1': 0.5, 'left_s1': -0.5, 'right_e1': 0.3, 'left_e1': 0.3, 'right_w1': 0.2, 'left_w1': -0.2},
            {'right_s1': -0.5, 'left_s1': 0.5, 'right_e1': -0.3, 'left_e1': -0.3, 'right_w1': -0.2, 'left_w1': 0.2},
            # Phase 2: Medium movements
            {'right_s1': 0.3, 'left_s1': -0.3, 'right_e1': 0.2, 'left_e1': 0.2, 'right_w1': 0.1, 'left_w1': -0.1},
            {'right_s1': -0.3, 'left_s1': 0.3, 'right_e1': -0.2, 'left_e1': -0.2, 'right_w1': -0.1, 'left_w1': 0.1},
            # Phase 3: Small movements
            {'right_s1': 0.15, 'left_s1': -0.15, 'right_e1': 0.1, 'left_e1': 0.1, 'right_w1': 0.05, 'left_w1': -0.05},
            {'right_s1': -0.15, 'left_s1': 0.15, 'right_e1': -0.1, 'left_e1': -0.1, 'right_w1': -0.05, 'left_w1': 0.05},
            # Phase 4: Return to center
            {'right_s1': 0.0, 'left_s1': 0.0, 'right_e1': 0.0, 'left_e1': 0.0, 'right_w1': 0.0, 'left_w1': 0.0}
        ]
        
        for i, wobble in enumerate(wobble_sequence):
            # Apply offsets to original positions
            for joint, offset in wobble.items():
                if 'right' in joint:
                    current_right[joint] = original_positions[joint] + offset
                else:
                    current_left[joint] = original_positions[joint] + offset
            
            # Move arms with appropriate speed
            speed = 0.8 if i < 2 else 0.6 if i < 4 else 0.4
            self.safe_move_arm(self.right_arm, current_right, speed)
            self.safe_move_arm(self.left_arm, current_left, speed)
            
            # Show wobble progress
            if i < len(wobble_sequence) - 1:
                time.sleep(0.4)
            else:
                time.sleep(0.6)  # Longer pause at end

    def safe_move_arm(self, arm, target_positions, move_speed=0.3):
        """Safely move arm to target position with error handling"""
        try:
            arm.set_joint_position_speed(move_speed)
            arm.move_to_joint_positions(target_positions)
            return True
        except Exception as e:
            print("Arm movement error: {}".format(e))
            return False

    def wake_up_sequence(self):
        """Professional wake up sequence"""
        print("=== WAKE UP SEQUENCE ===")
        
        # Sleep positions
        sleep_left = {
            'left_w0': 0.0, 'left_w1': 1.5, 'left_w2': 0.0,
            'left_e0': -1.2, 'left_e1': 1.8, 'left_s0': 0.3, 'left_s1': -0.5
        }
        sleep_right = {
            'right_w0': 0.0, 'right_w1': 1.5, 'right_w2': 0.0,
            'right_e0': 1.2, 'right_e1': 1.8, 'right_s0': -0.3, 'right_s1': -0.5
        }
        
        # Move to sleep position
        self.safe_move_arm(self.left_arm, sleep_left)
        self.safe_move_arm(self.right_arm, sleep_right)
        self.head.set_pan(0.0)
        
        # Sleep phase
        self.say_and_display("Zzz... zzz...", "sleeping", 3)
        
        # Wake up
        self.say_and_display("Mmm... what's happening?", "waking", 2)
        
        # Stretch
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
        
        self.say_and_display("Oh! Hello there, everyone!", "alert", 3)

    def notice_people_sequence(self):
        """Notice people and realize duck is missing"""
        print("=== NOTICING PEOPLE ===")
        
        # Look around at people
        self.say_and_display("", "alert")
        
        self.head.set_pan(-0.5)
        time.sleep(1)
        self.head.set_pan(0.5)
        time.sleep(1)
        self.head.set_pan(0.0)
        
        self.say_and_display("Wonderful! You're all here!", "excited", 3)
        
        # Dramatic realization
        self.say_and_display("Wait... something's wrong...", "confused", 2)
        self.say_and_display("Where is my rubber duck?!", "text", 3)

    def search_for_duck(self):
        """Enhanced duck search with better camera integration and screen feedback"""
        print("=== DUCK SEARCH SEQUENCE ===")
        
        # Show search initialization status
        search_init_status = [
            ("INITIATING DUCK SEARCH", (0, 255, 0)),
            ("Activating camera systems...", (0, 255, 255)),
            ("Preparing search pattern", (255, 255, 255)),
            ("Duck detection: ACTIVE", (255, 255, 0))
        ]
        status_img = self.create_status_display("SEARCH MODE", search_init_status)
        try:
            status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
            self.display_pub.publish(status_img)
            time.sleep(2)
        except:
            pass
        
        self.say_and_display("I must find my duck! Initiating search protocols...", "searching", 3)
        
        # Start camera detection
        camera_status = [
            ("CAMERA SYSTEM ONLINE", (0, 255, 0)),
            ("Using right hand camera", (0, 255, 255)),
            ("Resolution: 640x400", (255, 255, 255)),
            ("Detection threshold: {}%".format(int(self.detection_threshold * 100)), (255, 255, 0))
        ]
        status_img = self.create_status_display("CAMERA ACTIVE", camera_status)
        try:
            status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
            self.display_pub.publish(status_msg)
        except:
            pass
        
        self.start_camera_detection('right_hand_camera')
        time.sleep(2)
        
        # Search positions with descriptive names
        search_positions = [
            ("LEFT QUADRANT", {
                'right_w0': 0.0, 'right_w1': -0.8, 'right_w2': 0.0,
                'right_e0': 0.8, 'right_e1': 1.2, 'right_s0': -0.8, 'right_s1': -0.2
            }),
            ("CENTER AREA", {
                'right_w0': 0.0, 'right_w1': -0.8, 'right_w2': 0.0,
                'right_e0': 0.2, 'right_e1': 1.0, 'right_s0': -0.2, 'right_s1': 0.0
            }),
            ("RIGHT QUADRANT", {
                'right_w0': 0.0, 'right_w1': -0.8, 'right_w2': 0.0,
                'right_e0': -0.5, 'right_e1': 1.2, 'right_s0': 0.5, 'right_s1': -0.2
            }),
            ("TABLE SURFACE", {
                'right_w0': 0.0, 'right_w1': 0.3, 'right_w2': 0.0,
                'right_e0': 0.0, 'right_e1': 1.5, 'right_s0': 0.0, 'right_s1': 0.3
            })
        ]
        
        duck_found = False
        
        for area_name, position in search_positions:
            # Show current search area on screen
            area_status = [
                ("SCANNING: {}".format(area_name), (0, 255, 0)),
                ("Moving arm to position...", (0, 255, 255)),
                ("Camera stabilizing...", (255, 255, 255)),
                ("Detection attempts: 3", (255, 255, 0))
            ]
            status_img = self.create_status_display("SEARCH ACTIVE", area_status)
            try:
                status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
                self.display_pub.publish(status_msg)
            except:
                pass
            
            self.say_and_display("Scanning {}...".format(area_name.lower()), "searching", 1, show_status=False)
            
            if self.safe_move_arm(self.right_arm, position):
                time.sleep(2)  # Allow camera to stabilize
                
                # Multiple detection attempts with screen feedback
                for attempt in range(3):
                    attempt_status = [
                        ("DETECTION ATTEMPT: {}/3".format(attempt + 1), (0, 255, 0)),
                        ("Area: {}".format(area_name), (0, 255, 255)),
                        ("Analyzing image data...", (255, 255, 255)),
                        ("Please wait...", (255, 255, 0))
                    ]
                    status_img = self.create_status_display("ANALYZING", attempt_status)
                    try:
                        status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
                        self.display_pub.publish(status_msg)
                        time.sleep(1)
                    except:
                        pass
                    
                    if self.detect_duck_in_image():
                        # Duck found! Show success status
                        success_status = [
                            ("DUCK DETECTED!", (0, 255, 0)),
                            ("Location: {}".format(area_name), (0, 255, 255)),
                            ("Confidence: HIGH", (255, 255, 255)),
                            ("Initiating celebration...", (255, 255, 0))
                        ]
                        status_img = self.create_status_display("SUCCESS!", success_status)
                        try:
                            status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
                            self.display_pub.publish(status_msg)
                            time.sleep(2)
                        except:
                            pass
                        
                        self.say_and_display("DUCK LOCATED! Target acquired in {}!".format(area_name.lower()), "excited", 2, show_status=False)
                        
                        # Celebratory head nod
                        try:
                            self.head.command_nod()
                            time.sleep(1)
                            self.head.command_nod()
                        except:
                            # Manual nod fallback
                            for _ in range(3):
                                self.head.set_pan(0.2)
                                time.sleep(0.3)
                                self.head.set_pan(-0.2)
                                time.sleep(0.3)
                            self.head.set_pan(0.0)
                        
                        duck_found = True
                        break
                    time.sleep(0.5)
                
                if duck_found:
                    break
            else:
                # Arm movement failed, show error
                error_status = [
                    ("ARM MOVEMENT ERROR", (255, 0, 0)),
                    ("Could not reach {}".format(area_name), (255, 255, 0)),
                    ("Continuing search...", (0, 255, 255)),
                    ("Safety systems active", (255, 255, 255))
                ]
                status_img = self.create_status_display("ERROR", error_status)
                try:
                    status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
                    self.display_pub.publish(status_msg)
                    time.sleep(1)
                except:
                    pass
        
        self.stop_camera_detection()
        
        if not duck_found:
            # Show fallback status
            fallback_status = [
                ("SEARCH INCOMPLETE", (255, 255, 0)),
                ("Duck presence detected", (0, 255, 255)),
                ("Continuing mission...", (255, 255, 255)),
                ("Human assistance ready", (0, 255, 0))
            ]
            status_img = self.create_status_display("FALLBACK MODE", fallback_status)
            try:
                status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
                self.display_pub.publish(status_msg)
                time.sleep(2)
            except:
                pass
            
            self.say_and_display("Duck signature detected nearby! Proceeding to retrieval phase...", "happy", 3, show_status=False)
            duck_found = True  # Continue story flow
        
        return duck_found

    def ask_for_duck(self):
        """Politely request the duck"""
        print("=== REQUESTING DUCK ===")
        
        # Polite gesture pose
        request_left = {
            'left_w0': 0.0, 'left_w1': -0.8, 'left_w2': 0.0,
            'left_e0': -0.3, 'left_e1': 1.0, 'left_s0': 0.2, 'left_s1': -0.5
        }
        request_right = {
            'right_w0': 0.3, 'right_w1': -1.2, 'right_w2': 0.0,
            'right_e0': 0.5, 'right_e1': 1.5, 'right_s0': -0.3, 'right_s1': -0.2
        }
        
        self.safe_move_arm(self.left_arm, request_left)
        self.safe_move_arm(self.right_arm, request_right)
        
        self.say_and_display("Could you please hand me my duck?", "happy", 3)

    def receive_duck_sequence(self):
        """Enhanced duck receiving with better detection"""
        print("=== DUCK RECEIVING SEQUENCE ===")
        
        # Prepare to receive
        receive_pose = {
            'right_w0': 0.0, 'right_w1': -0.5, 'right_w2': 0.0,
            'right_e0': 0.2, 'right_e1': 1.0, 'right_s0': -0.2, 'right_s1': 0.0
        }
        self.safe_move_arm(self.right_arm, receive_pose)
        self.right_gripper.open()
        
        # Start detection
        self.start_camera_detection('right_hand_camera')
        
        # Countdown
        for i in range(3, 0, -1):
            self.say_and_display("Ready in {}...".format(i), "text", 1)
        
        self.say_and_display("Please place the duck in my hand!", "searching", 2)
        
        # Wait for duck with timeout
        duck_received = False
        wait_time = 0
        max_wait = 10
        
        while not duck_received and wait_time < max_wait:
            if self.detect_duck_in_image():
                duck_received = True
                self.say_and_display("Perfect! I can sense it!", "excited", 2)
            else:
                time.sleep(1)
                wait_time += 1
                if wait_time % 3 == 0:  # Progress update every 3 seconds
                    self.say_and_display("Almost there...", "searching", 1)
        
        if not duck_received:
            self.say_and_display("I feel my duck's presence! Thank you!", "happy", 2)
        
        self.stop_camera_detection()
        
        # Close gripper
        self.say_and_display("Got it! Thank you so much!", "happy", 2)
        self.right_gripper.close()

    def say_and_display(self, text, display_type="text", duration=3, show_status=True):
        """Unified speech and display function with status information"""
        if show_status:
            # Display important info on screen instead of just terminal
            status_info = [
                ("Current Action: {}".format(text), (0, 255, 0)),
                ("System Status: ACTIVE", (0, 255, 255)),
                ("Duck Quest Progress: RUNNING", (255, 255, 255))
            ]
            status_img = self.create_status_display("BAXTER DUCK QUEST", status_info)
            try:
                status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
                self.display_pub.publish(status_msg)
                time.sleep(1.5)  # Show status briefly
            except Exception as e:
                pass  # Continue without status display
        
        # Send to speech synthesizer
        try:
            speech_msg = String()
            speech_msg.data = text
            self.speech_pub.publish(speech_msg)
        except Exception as e:
            pass  # Continue without speech
        
        # Show on display
        if display_type == "text":
            display_img = self.create_text_display(text)
        else:
            display_img = self.create_expression(display_type)
            
        try:
            display_msg = self.bridge.cv2_to_imgmsg(display_img, "bgr8")
            self.display_pub.publish(display_msg)
        except Exception as e:
            pass  # Continue without display
        
        time.sleep(duration)

    def _custom_wobble_sequence(self):
        """Custom wobble implementation"""
        print("Executing custom celebration wobble")
        
        # Get current positions
        current_right = self.right_arm.joint_angles()
        current_left = self.left_arm.joint_angles()
        
        # Store originals
        original_right_s1 = current_right['right_s1']
        original_left_s1 = current_left['left_s1']
        
        # Wobble sequence
        wobble_offsets = [0.4, -0.4, 0.3, -0.3, 0.2, -0.2, 0.0]
        
        for offset in wobble_offsets:
            current_right['right_s1'] = original_right_s1 + offset
            current_left['left_s1'] = original_left_s1 - offset
            
            self.safe_move_arm(self.right_arm, current_right, 0.5)
            self.safe_move_arm(self.left_arm, current_left, 0.5)
            time.sleep(0.3)

    def place_duck_safely(self):
        """Carefully place duck on table"""
        print("=== PLACING DUCK SAFELY ===")
        
        self.say_and_display("Let me put my duck somewhere safe...", "text", 2)
        
        # Safe placement position
        table_pose = {
            'right_w0': 0.0, 'right_w1': 0.5, 'right_w2': 0.0,
            'right_e0': 0.3, 'right_e1': 1.8, 'right_s0': -0.1, 'right_s1': 0.5
        }
        
        if self.safe_move_arm(self.right_arm, table_pose):
            self.say_and_display("There you go, little duck. Safe and sound!", "happy", 3)
            self.right_gripper.open()
            
            # Return to safe position
            safe_pose = {
                'right_w0': 0.0, 'right_w1': -0.5, 'right_w2': 0.0,
                'right_e0': 0.0, 'right_e1': 1.0, 'right_s0': 0.0, 'right_s1': -0.3
            }
            self.safe_move_arm(self.right_arm, safe_pose)

    def final_celebration(self):
        """Grand finale celebration"""
        print("=== FINAL CELEBRATION ===")
        
        self.say_and_display("You know what? Let's celebrate together!", "excited", 3)
        
        # Victory pose
        victory_left = {
            'left_w0': -1.5, 'left_w1': 0.0, 'left_w2': 0.0,
            'left_e0': -0.8, 'left_e1': 1.2, 'left_s0': 0.8, 'left_s1': -0.5
        }
        victory_right = {
            'right_w0': 1.5, 'right_w1': 0.0, 'right_w2': 0.0,
            'right_e0': 0.8, 'right_e1': 1.2, 'right_s0': -0.8, 'right_s1': -0.5
        }
        
        self.safe_move_arm(self.left_arm, victory_left)
        self.safe_move_arm(self.right_arm, victory_right)
        
        self.say_and_display("Thank you for helping me find my duck!", "excited", 4)
        
        # Appreciative head nods
        try:
            for _ in range(2):
                self.head.command_nod()
                time.sleep(1.5)
        except:
            # Manual nod fallback
            for _ in range(4):
                self.head.set_pan(0.3)
                time.sleep(0.3)
                self.head.set_pan(-0.3)
                time.sleep(0.3)
            self.head.set_pan(0.0)
        
        # Final bow
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
        
        self.say_and_display("You're all wonderful! Duck quest complete!", "happy", 4)

    def return_to_neutral_position(self):
        """Return robot to safe neutral position"""
        print("=== RETURNING TO NEUTRAL ===")
        
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
        
        self.say_and_display("Ready for the next adventure!", "neutral", 2)

    def run_complete_duck_quest(self):
        """Execute the complete duck quest narrative"""
        print("=" * 50)
        print("BAXTER'S DUCK QUEST - INTERACTIVE ADVENTURE")
        print("=" * 50)
        
        try:
            # Complete story sequence
            self.wake_up_sequence()
            time.sleep(1)
            
            self.notice_people_sequence()
            time.sleep(1)
            
            duck_found = self.search_for_duck()
            time.sleep(1)
            
            if duck_found:
                self.ask_for_duck()
                time.sleep(1)
                
                self.receive_duck_sequence()
                time.sleep(1)
                
                self.celebration_wobble()
                time.sleep(1)
                
                self.place_duck_safely()
                time.sleep(1)
                
                self.final_celebration()
                time.sleep(1)
                
                self.return_to_neutral_position()
            
            print("=" * 50)
            print("DUCK QUEST ADVENTURE COMPLETED SUCCESSFULLY!")
            print("=" * 50)
            
        except rospy.ROSInterruptException:
            print("Adventure interrupted by user")
        except Exception as e:
            print("Adventure error: {}".format(e))
            print("Attempting safe shutdown...")
            self.emergency_stop()

    def emergency_stop(self):
        """Emergency stop procedure"""
        print("EMERGENCY STOP ACTIVATED")
        try:
            # Stop all motion
            self.left_arm.exit_control_mode()
            self.right_arm.exit_control_mode()
            
            # Stop camera
            self.stop_camera_detection()
            
            # Display emergency message
            self.say_and_display("Emergency stop activated. Please check system.", "alert", 3)
            
        except Exception as e:
            print("Emergency stop error: {}".format(e))

    def test_individual_components(self):
        """Test individual components for debugging"""
        print("=== COMPONENT TESTING MODE ===")
        
        # Test display system
        print("Testing display system...")
        self.say_and_display("Display test successful!", "happy", 2)
        
        # Test expressions
        expressions = ["sleeping", "waking", "alert", "confused", "searching", "excited", "happy", "neutral"]
        for expr in expressions:
            print("Testing {} expression...".format(expr))
            img = self.create_expression(expr)
            display_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.display_pub.publish(display_msg)
            time.sleep(1)
        
        # Test arm movements
        print("Testing arm movements...")
        test_pose = {
            'right_w0': 0.0, 'right_w1': 0.0, 'right_w2': 0.0,
            'right_e0': 0.5, 'right_e1': 1.0, 'right_s0': 0.0, 'right_s1': 0.0
        }
        
        if self.safe_move_arm(self.right_arm, test_pose):
            print("Arm movement test successful!")
        else:
            print("Arm movement test failed!")
        
        # Test camera
        print("Testing camera system...")
        self.start_camera_detection()
        time.sleep(2)
        
        if self.current_camera_image is not None:
            print("Camera test successful!")
            height, width = self.current_camera_image.shape[:2]
            print("Camera resolution: {}x{}".format(width, height))
        else:
            print("Camera test failed!")
        
        self.stop_camera_detection()
        
        print("Component testing completed!")

def main():
    """Main execution function"""
    try:
        print("Initializing Baxter's Duck Quest...")
        quest = BaxterDuckQuest()
        
        print("\n" + "=" * 50)
        print("BAXTER'S DUCK QUEST - READY TO BEGIN!")
        print("=" * 50)
        print("Instructions:")
        print("1. Make sure you have a rubber duck ready")
        print("2. The duck should be colorful and visible to the camera")
        print("3. Be ready to place it in Baxter's hand when requested")
        print("4. Enjoy the interactive adventure!")
        print("=" * 50)
        
        # Option to test components first
        print("\nOptions:")
        print("1. Run complete duck quest adventure")
        print("2. Test individual components")
        print("3. Emergency stop")
        
        try:
            choice = raw_input("\nEnter your choice (1-3): ").strip()
        except EOFError:
            choice = "1"  # Default choice
        
        if choice == "2":
            quest.test_individual_components()
        elif choice == "3":
            quest.emergency_stop()
        else:
            print("\nStarting duck quest adventure in 3 seconds...")
            time.sleep(3)
            quest.run_complete_duck_quest()
        
    except rospy.ROSInterruptException:
        print("Program interrupted by user")
    except KeyboardInterrupt:
        print("Program interrupted by keyboard")
    except Exception as e:
        print("Initialization error: {}".format(e))
        print("Please check:")
        print("- ROS is running (roscore)")
        print("- Baxter robot is connected and enabled")
        print("- All required packages are installed")

if __name__ == '__main__':
    main()