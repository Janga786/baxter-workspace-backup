#!/usr/bin/env python2.7

"""
Baxter Communication Library
Handles display, speech, and expressions for the master demo
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import time
from baxter_face_expressions import create_digital_expression


class BaxterCommunication(object):
    """Handles all communication - display, speech, and expressions"""
    
    def __init__(self):
        """Initialize communication systems"""
        self.display_pub = rospy.Publisher('/robot/xdisplay', Image, queue_size=1)
        self.speech_pub = rospy.Publisher('/robot/say', String, queue_size=1)
        self.bridge = CvBridge()
        
        # Allow publishers to initialize
        time.sleep(1.0)
    
    def say_and_display(self, text, expression="neutral", duration=3, show_text=True):
        """Unified speech and display function"""
        rospy.loginfo("Baxter says: {}".format(text))
        
        # Send to speech synthesizer
        try:
            speech_msg = String()
            speech_msg.data = text
            self.speech_pub.publish(speech_msg)
        except Exception as e:
            rospy.logwarn("Speech error: {}".format(e))
        
        # Show on display
        if show_text:
            display_img = self.create_text_display(text)
        else:
            display_img = create_digital_expression(expression)
            
        try:
            display_msg = self.bridge.cv2_to_imgmsg(display_img, "bgr8")
            self.display_pub.publish(display_msg)
        except Exception as e:
            rospy.logwarn("Display error: {}".format(e))
        
        time.sleep(duration)
    
    def show_expression(self, expression_type, duration=2):
        """Show facial expression without speech"""
        try:
            img = create_digital_expression(expression_type)
            display_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.display_pub.publish(display_msg)
            time.sleep(duration)
        except Exception as e:
            rospy.logwarn("Expression error: {}".format(e))
    
    def create_text_display(self, text, bg_color=(0, 0, 0)):
        """Create text display for the head screen"""
        width, height = 1024, 600
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = bg_color
        
        # Smart text wrapping
        max_chars_per_line = 20
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
        
        # Draw text
        for i, line in enumerate(lines):
            text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_DUPLEX, 2, 3)[0]
            text_x = (width - text_size[0]) // 2
            text_y = start_y + i * line_height
            
            # Clean white text
            cv2.putText(img, line, (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX, 2, 
                       (255, 255, 255), 3)
        
        return img
    
    def create_status_display(self, title, status_info, bg_color=(20, 20, 40)):
        """Create status display with information"""
        width, height = 1024, 600
        img = np.zeros((height, width, 3), dtype=np.uint8)
        img[:] = bg_color
        
        # Colors
        bright_green = (0, 255, 0)
        cyan = (0, 255, 255)
        white = (255, 255, 255)
        
        # Title at top
        title_size = cv2.getTextSize(title, cv2.FONT_HERSHEY_DUPLEX, 1.5, 2)[0]
        title_x = (width - title_size[0]) // 2
        cv2.putText(img, title, (title_x, 80), cv2.FONT_HERSHEY_DUPLEX, 1.5, 
                   bright_green, 2)
        
        # Underline
        cv2.line(img, (title_x, 90), (title_x + title_size[0], 90), cyan, 2)
        
        # Status information
        y_offset = 150
        for i, info in enumerate(status_info):
            if isinstance(info, tuple):
                text, color = info
            else:
                text, color = info, white
                
            cv2.putText(img, text, (50, y_offset + i * 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
        
        return img
    
    def show_status(self, title, status_info, duration=3):
        """Show status display"""
        try:
            status_img = self.create_status_display(title, status_info)
            status_msg = self.bridge.cv2_to_imgmsg(status_img, "bgr8")
            self.display_pub.publish(status_msg)
            time.sleep(duration)
        except Exception as e:
            rospy.logwarn("Status display error: {}".format(e))
    
    def introduction_sequence(self):
        """Introduction with various expressions"""
        intro_messages = [
            ("Hello everyone! I'm Baxter!", "excited", 3),
            ("Welcome to my demonstration!", "happy", 3),
            ("Let me show you what I can do!", "alert", 3)
        ]
        
        for message, expression, duration in intro_messages:
            self.say_and_display(message, expression, duration, show_text=True)
    
    def conclusion_sequence(self):
        """Conclusion with gratitude"""
        conclusion_messages = [
            ("Thank you for watching my demo!", "happy", 3),
            ("I hope you enjoyed it!", "excited", 3),
            ("Until next time, goodbye!", "waving", 3)
        ]
        
        for message, expression, duration in conclusion_messages:
            self.say_and_display(message, expression, duration, show_text=True)