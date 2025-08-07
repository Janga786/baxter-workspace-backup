#!/usr/bin/env python3

import subprocess
import time

def fix_rviz_display():
    """Fix RViz display issues"""
    print("🔧 Fixing RViz Display...")
    
    try:
        # Kill any existing RViz instances
        subprocess.run([
            'docker', 'exec', 'baxter_moveit', 
            'pkill', '-f', 'rviz2'
        ], capture_output=True)
        
        time.sleep(2)
        
        # Restart RViz with proper configuration
        print("🚀 Restarting RViz with MoveIt configuration...")
        
        cmd = [
            'docker', 'exec', '-d', 'baxter_moveit',
            'bash', '-c',
            '''cd /shared_ws && 
               source /opt/ros/humble/setup.bash && 
               source install/setup.bash && 
               rviz2 -d /opt/ros/humble/share/moveit_setup_assistant/rviz/moveit_default.rviz'''
        ]
        
        subprocess.run(cmd, check=True)
        
        print("✅ RViz restarted with MoveIt configuration")
        print("📋 In RViz:")
        print("   1. Add Robot Model display if not present")
        print("   2. Set Fixed Frame to 'base' or 'world'")
        print("   3. Add MotionPlanning display")
        print("   4. Robot should now be visible")
        
        time.sleep(3)
        
        # Check if it's running
        result = subprocess.run([
            'docker', 'exec', 'baxter_moveit',
            'pgrep', '-f', 'rviz2'
        ], capture_output=True, text=True)
        
        if result.stdout.strip():
            print("✅ RViz is running")
            return True
        else:
            print("❌ RViz may not have started properly")
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == '__main__':
    fix_rviz_display()