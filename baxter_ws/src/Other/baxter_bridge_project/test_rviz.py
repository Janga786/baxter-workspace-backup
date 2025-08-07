#!/usr/bin/env python3

import subprocess
import os
import time

def test_rviz():
    """Test if RViz can launch"""
    
    print("Testing RViz launch...")
    
    try:
        # Test if display is available
        result = subprocess.run(['xdpyinfo'], capture_output=True, text=True)
        if result.returncode != 0:
            print("❌ No X11 display available")
            print("Solutions:")
            print("1. If using SSH: ssh -X username@hostname")
            print("2. If local: export DISPLAY=:0")
            print("3. Install X11 server if needed")
            return False
            
        print("✅ X11 display available")
        
        # Try to launch RViz in container
        print("Launching RViz...")
        cmd = [
            'docker', 'exec', '-d', 'baxter_moveit',
            'bash', '-c', 
            'cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && rviz2'
        ]
        
        subprocess.run(cmd, check=True)
        print("✅ RViz launch command sent")
        
        time.sleep(3)
        
        # Check if RViz is running
        result = subprocess.run([
            'docker', 'exec', 'baxter_moveit',
            'bash', '-c', 'ps aux | grep rviz2 | grep -v grep'
        ], capture_output=True, text=True)
        
        if result.stdout.strip():
            print("✅ RViz is running")
            return True
        else:
            print("❌ RViz failed to start")
            return False
            
    except Exception as e:
        print(f"❌ Error testing RViz: {e}")
        return False

if __name__ == '__main__':
    test_rviz()