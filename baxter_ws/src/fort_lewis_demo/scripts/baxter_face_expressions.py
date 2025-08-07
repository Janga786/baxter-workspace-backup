#!/usr/bin/env python2.7

"""
Baxter Digital Facial Expressions
Clean, digital-looking expressions focused on eyes and eyebrows
Black and white aesthetic with detailed geometric patterns
"""

import cv2
import numpy as np
import math
import time

def create_digital_expression(expression_type):
    """Create clean, digital facial expressions for Baxter's display"""
    width, height = 1024, 600
    
    # Pure black background for digital aesthetic
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Define eye positions
    left_eye_x = width // 3
    right_eye_x = 2 * width // 3
    eye_y = height // 2 - 50
    
    # Define colors (high contrast for digital look)
    white = (255, 255, 255)
    light_gray = (200, 200, 200)
    medium_gray = (128, 128, 128)
    dark_gray = (80, 80, 80)
    black = (0, 0, 0)
    
    if expression_type == "sleeping":
        # Simple closed eyes with digital eyelash pattern
        for eye_x in [left_eye_x, right_eye_x]:
            # Main eyelid line
            cv2.line(img, (eye_x - 80, eye_y), (eye_x + 80, eye_y), white, 8)
            
            # Digital eyelash pattern
            for i in range(-6, 7):
                lash_x = eye_x + i * 12
                lash_length = 20 + abs(i) * 2
                cv2.line(img, (lash_x, eye_y - 5), (lash_x, eye_y - lash_length), light_gray, 2)
            
            # Small corner accents
            cv2.circle(img, (eye_x - 85, eye_y), 4, medium_gray, -1)
            cv2.circle(img, (eye_x + 85, eye_y), 4, medium_gray, -1)
    
    elif expression_type == "waking":
        # Half-open eyes with geometric patterns
        for eye_x in [left_eye_x, right_eye_x]:
            # Half-circle eye shape
            cv2.ellipse(img, (eye_x, eye_y), (70, 35), 0, 0, 180, white, -1)
            cv2.ellipse(img, (eye_x, eye_y), (70, 35), 0, 0, 180, black, 4)
            
            # Simple pupil
            cv2.circle(img, (eye_x, eye_y + 10), 20, black, -1)
            
            # Digital highlight
            cv2.rectangle(img, (eye_x - 8, eye_y - 5), (eye_x + 8, eye_y + 5), white, -1)
    
    elif expression_type == "alert":
        # Wide, geometric eyes with detailed patterns
        for eye_x in [left_eye_x, right_eye_x]:
            # Outer eye circle
            cv2.circle(img, (eye_x, eye_y), 80, white, -1)
            cv2.circle(img, (eye_x, eye_y), 80, black, 6)
            
            # Inner eye detail
            cv2.circle(img, (eye_x, eye_y), 65, light_gray, 2)
            cv2.circle(img, (eye_x, eye_y), 50, medium_gray, 1)
            
            # Sharp, focused pupil
            cv2.circle(img, (eye_x, eye_y), 30, black, -1)
            
            # Digital crosshair in pupil
            cv2.line(img, (eye_x - 15, eye_y), (eye_x + 15, eye_y), dark_gray, 2)
            cv2.line(img, (eye_x, eye_y - 15), (eye_x, eye_y + 15), dark_gray, 2)
            
            # Corner accents
            for angle in [45, 135, 225, 315]:
                end_x = int(eye_x + 90 * math.cos(math.radians(angle)))
                end_y = int(eye_y + 90 * math.sin(math.radians(angle)))
                cv2.line(img, (eye_x, eye_y), (end_x, end_y), light_gray, 2)
        
        # Alert eyebrows - sharp and raised
        for eye_x in [left_eye_x, right_eye_x]:
            brow_y = eye_y - 120
            # Angular eyebrow
            points = np.array([[eye_x - 60, brow_y + 20], [eye_x - 20, brow_y], 
                              [eye_x + 20, brow_y], [eye_x + 60, brow_y + 20]], np.int32)
            cv2.polylines(img, [points], False, white, 8)
    
    elif expression_type == "confused":
        # Asymmetric eyes with question mark patterns
        # Left eye - normal
        cv2.circle(img, (left_eye_x, eye_y), 70, white, -1)
        cv2.circle(img, (left_eye_x, eye_y), 70, black, 4)
        cv2.circle(img, (left_eye_x - 10, eye_y), 25, black, -1)
        
        # Right eye - squinted
        cv2.ellipse(img, (right_eye_x, eye_y + 10), (65, 45), 0, 0, 360, white, -1)
        cv2.ellipse(img, (right_eye_x, eye_y + 10), (65, 45), 0, 0, 360, black, 4)
        cv2.circle(img, (right_eye_x + 10, eye_y + 10), 25, black, -1)
        
        # Confused eyebrows - one raised, one furrowed
        # Left eyebrow - raised
        cv2.line(img, (left_eye_x - 50, eye_y - 100), (left_eye_x + 50, eye_y - 130), white, 8)
        # Right eyebrow - furrowed
        cv2.line(img, (right_eye_x - 50, eye_y - 80), (right_eye_x + 50, eye_y - 100), white, 8)
        
        # Digital question marks
        font_scale = 2
        cv2.putText(img, "?", (width // 2 - 20, eye_y - 160), cv2.FONT_HERSHEY_SIMPLEX, 
                   font_scale, light_gray, 6)
    
    elif expression_type == "searching":
        # Scanning eyes with digital overlays
        for eye_x in [left_eye_x, right_eye_x]:
            # Base eye
            cv2.circle(img, (eye_x, eye_y), 75, white, -1)
            cv2.circle(img, (eye_x, eye_y), 75, black, 4)
            
            # Scanning pupil with crosshairs
            cv2.circle(img, (eye_x + 15, eye_y), 35, black, -1)
            cv2.circle(img, (eye_x + 15, eye_y), 35, light_gray, 2)
            
            # Digital scanning lines
            for i in range(3):
                line_y = eye_y - 20 + i * 20
                cv2.line(img, (eye_x - 60, line_y), (eye_x + 60, line_y), medium_gray, 1)
            
            # Corner brackets for scanning effect
            bracket_size = 15
            # Top-left
            cv2.line(img, (eye_x - 75, eye_y - 75), (eye_x - 75 + bracket_size, eye_y - 75), light_gray, 3)
            cv2.line(img, (eye_x - 75, eye_y - 75), (eye_x - 75, eye_y - 75 + bracket_size), light_gray, 3)
            # Top-right  
            cv2.line(img, (eye_x + 75, eye_y - 75), (eye_x + 75 - bracket_size, eye_y - 75), light_gray, 3)
            cv2.line(img, (eye_x + 75, eye_y - 75), (eye_x + 75, eye_y - 75 + bracket_size), light_gray, 3)
            # Bottom-left
            cv2.line(img, (eye_x - 75, eye_y + 75), (eye_x - 75 + bracket_size, eye_y + 75), light_gray, 3)
            cv2.line(img, (eye_x - 75, eye_y + 75), (eye_x - 75, eye_y + 75 - bracket_size), light_gray, 3)
            # Bottom-right
            cv2.line(img, (eye_x + 75, eye_y + 75), (eye_x + 75 - bracket_size, eye_y + 75), light_gray, 3)
            cv2.line(img, (eye_x + 75, eye_y + 75), (eye_x + 75, eye_y + 75 - bracket_size), light_gray, 3)
        
        # Focused eyebrows
        for eye_x in [left_eye_x, right_eye_x]:
            brow_y = eye_y - 110
            cv2.line(img, (eye_x - 40, brow_y + 15), (eye_x + 40, brow_y - 15), white, 10)
    
    elif expression_type == "excited":
        # Star-burst eyes with energy patterns
        for eye_x in [left_eye_x, right_eye_x]:
            # Star pattern in eye
            star_points = 8
            outer_radius = 80
            inner_radius = 40
            
            # Create star points
            points = []
            for i in range(star_points * 2):
                angle = i * math.pi / star_points
                radius = outer_radius if i % 2 == 0 else inner_radius
                x = int(eye_x + radius * math.cos(angle))
                y = int(eye_y + radius * math.sin(angle))
                points.append([x, y])
            
            star_array = np.array([points], dtype=np.int32)
            cv2.fillPoly(img, star_array, white)
            cv2.polylines(img, star_array, True, black, 4)
            
            # Central pupil
            cv2.circle(img, (eye_x, eye_y), 25, black, -1)
            cv2.circle(img, (eye_x, eye_y), 12, white, -1)
        
        # Excited eyebrows - high and curved
        for eye_x in [left_eye_x, right_eye_x]:
            brow_y = eye_y - 140
            cv2.ellipse(img, (eye_x, brow_y), (60, 30), 0, 0, 180, white, 12)
    
    elif expression_type == "happy":
        # Curved happy eyes with clean lines
        for eye_x in [left_eye_x, right_eye_x]:
            # Curved eye shape (smile-like)
            cv2.ellipse(img, (eye_x, eye_y), (70, 40), 0, 0, 180, white, -1)
            cv2.ellipse(img, (eye_x, eye_y), (70, 40), 0, 0, 180, black, 6)
            
            # Happy highlight
            cv2.ellipse(img, (eye_x, eye_y - 10), (30, 15), 0, 0, 180, light_gray, -1)
        
        # Happy eyebrows - gently curved
        for eye_x in [left_eye_x, right_eye_x]:
            brow_y = eye_y - 100
            cv2.ellipse(img, (eye_x, brow_y), (50, 20), 0, 180, 360, white, 8)
    
    else:  # neutral/default
        # Clean, simple geometric eyes
        for eye_x in [left_eye_x, right_eye_x]:
            # Outer circle
            cv2.circle(img, (eye_x, eye_y), 70, white, -1)
            cv2.circle(img, (eye_x, eye_y), 70, black, 4)
            
            # Inner detail circles
            cv2.circle(img, (eye_x, eye_y), 55, light_gray, 1)
            cv2.circle(img, (eye_x, eye_y), 40, medium_gray, 1)
            
            # Pupil
            cv2.circle(img, (eye_x, eye_y), 25, black, -1)
            
            # Digital highlight - rectangular for clean look
            cv2.rectangle(img, (eye_x - 8, eye_y - 12), (eye_x + 8, eye_y - 4), white, -1)
        
        # Neutral eyebrows - clean horizontal lines
        for eye_x in [left_eye_x, right_eye_x]:
            brow_y = eye_y - 100
            cv2.line(img, (eye_x - 40, brow_y), (eye_x + 40, brow_y), white, 6)
    
    return img

def create_text_display(text, bg_color=(0, 0, 0)):
    """Create a clean text display for the head screen"""
    width, height = 1024, 600
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = bg_color
    
    # Split text into lines if too long
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
    
    # Calculate starting position
    line_height = 80
    total_height = len(lines) * line_height
    start_y = (height - total_height) // 2 + 60
    
    # Draw each line with clean digital font
    for i, line in enumerate(lines):
        text_size = cv2.getTextSize(line, cv2.FONT_HERSHEY_DUPLEX, 2, 3)[0]
        text_x = (width - text_size[0]) // 2
        text_y = start_y + i * line_height
        
        # Clean white text on black background
        cv2.putText(img, line, (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX, 2, (255, 255, 255), 3)
    
    return img

# Example usage function
def demo_expressions():
    """Demo all the digital expressions"""
    expressions = [
        "neutral",
        "sleeping", 
        "waking",
        "alert",
        "confused",
        "searching", 
        "excited",
        "happy"
    ]
    
    print("Creating digital expressions...")
    
    for expr in expressions:
        img = create_digital_expression(expr)
        print("Created {} expression".format(expr))
        
        # In your actual code, you would publish this to Baxter's display:
        # display_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        # self.display_pub.publish(display_msg)
        
        time.sleep(1)  # Simulate display time

if __name__ == "__main__":
    demo_expressions()