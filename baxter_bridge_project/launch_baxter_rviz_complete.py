#!/usr/bin/env python3

import os
import subprocess
import time

def launch_complete_baxter_rviz():
    """Launch complete Baxter system with RViz"""
    
    print("🤖 Launching Complete Baxter RViz System")
    print("========================================")
    
    # Step 1: Launch robot state publisher with Baxter URDF
    print("📡 Starting robot description publisher...")
    
    subprocess.Popen([
        'docker', 'exec', 'baxter_moveit', 'bash', '-c',
        '''cd /shared_ws && 
        source /opt/ros/humble/setup.bash && 
        source install/setup.bash && 
        ros2 run robot_state_publisher robot_state_publisher \
        --ros-args -p robot_description:="$(xacro /shared_ws/src/baxter_moveit_config/urdf/baxter.urdf.xacro)" &'''
    ])
    
    time.sleep(3)
    
    # Step 2: Launch joint state publisher  
    print("🦾 Starting joint state publisher...")
    subprocess.Popen([
        'docker', 'exec', 'baxter_moveit', 'bash', '-c',
        '''cd /shared_ws && 
        source /opt/ros/humble/setup.bash && 
        ros2 run joint_state_publisher joint_state_publisher &'''
    ])
    
    time.sleep(2)
    
    # Step 3: Launch MoveIt move_group
    print("🧠 Starting MoveIt move_group...")
    subprocess.Popen([
        'docker', 'exec', 'baxter_moveit', 'bash', '-c',
        '''cd /shared_ws && 
        source /opt/ros/humble/setup.bash && 
        source install/setup.bash && 
        ros2 launch baxter_moveit_config move_group.launch.py &'''
    ])
    
    time.sleep(5)
    
    # Step 4: Launch RViz with motion planning
    print("🎮 Starting RViz with motion planning...")
    subprocess.Popen([
        'docker', 'exec', 'baxter_moveit', 'bash', '-c',
        f'''cd /shared_ws && 
        source /opt/ros/humble/setup.bash && 
        source install/setup.bash && 
        export DISPLAY={os.environ.get('DISPLAY', ':0')} &&
        ros2 run rviz2 rviz2 -d /shared_ws/src/baxter_moveit_config/launch/moveit.rviz &'''
    ])
    
    time.sleep(3)
    
    print("✅ Complete system launched!")
    print("")
    print("🎯 What should be visible in RViz:")
    print("- Baxter robot model (white/gray robot)")
    print("- Motion Planning panel on left")
    print("- Interactive markers on end-effector")
    print("")
    print("🎮 To control:")
    print("1. Select planning group: left_arm or right_arm")
    print("2. Drag orange markers to set goal")
    print("3. Click Plan, then Execute")
    
if __name__ == '__main__':
    launch_complete_baxter_rviz()