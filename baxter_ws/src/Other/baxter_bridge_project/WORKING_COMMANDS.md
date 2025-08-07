# 🚀 WORKING Baxter Commands - Use These!

## ✅ What's Currently Running:
- **MoveIt planning system** ✅
- **RViz visualization** ✅ 
- **Real Baxter connection** ✅

---

## 🎯 Quick Commands That Work RIGHT NOW:

### 1. Test Robot Movement (WORKS!)
```bash
./test_baxter_movement.sh
```
**Expected result**: Baxter's left arm waves back and forth

### 2. Run MoveIt Demo with Real Robot
```bash
docker exec -it baxter_moveit bash -c "cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 /shared_ws/demo_moveit_control.py"
```
**Expected result**: Plans with MoveIt, executes on real robot

### 3. Interactive RViz Control
RViz should already be open. If not:
```bash
python3 test_rviz.py
```

### 4. Development Environment
```bash
docker exec -it baxter_moveit bash
cd /shared_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
# Now you can write Python scripts!
```

---

## 🎮 Using RViz (Should be open now):

1. **Look for RViz window** - it should be open
2. **In Motion Planning panel**:
   - Planning Group: Select "left_arm" 
   - Goal State: Drag orange robot model
   - Click "Plan" button
   - Click "Execute" button

3. **If RViz isn't visible**:
   - Check taskbar/dock for RViz window
   - Alt+Tab to cycle through windows
   - Run: `python3 test_rviz.py`

---

## 💻 Write Your Own Code Template:

Save this as `my_baxter_script.py`:
```python
#!/usr/bin/env python3

import rclpy
import moveit_commander
import sys
import subprocess

def move_baxter():
    # Initialize
    rclpy.init()
    moveit_commander.roscpp_initialize(sys.argv)
    
    # Get arm group
    left_arm = moveit_commander.MoveGroupCommander("left_arm")
    
    # Define target position
    target = {
        'left_s0': 0.3, 'left_s1': -0.5, 'left_e0': 0.0, 'left_e1': 0.7,
        'left_w0': 0.0, 'left_w1': 1.2, 'left_w2': 0.0
    }
    
    # Plan movement
    left_arm.set_joint_value_target(target)
    plan = left_arm.plan()
    
    if plan[0]:
        print("✅ Planning successful!")
        
        # Extract positions from plan
        final_point = plan[1].joint_trajectory.points[-1]
        positions = list(final_point.positions)
        
        # Send to real robot
        cmd_script = f'''
import rospy
from sensor_msgs.msg import JointState
rospy.init_node('my_controller', anonymous=True)
pub = rospy.Publisher('/robot/limb/left/command_joint_position', JointState, queue_size=1)
rospy.sleep(1.0)
msg = JointState()
msg.header.stamp = rospy.Time.now()
msg.name = ['left_s0', 'left_s1', 'left_e0', 'left_e1', 'left_w0', 'left_w1', 'left_w2']
msg.position = {positions}
for _ in range(10):
    pub.publish(msg)
    rospy.sleep(0.1)
'''
        with open('/tmp/my_cmd.py', 'w') as f:
            f.write(cmd_script)
            
        subprocess.run([
            'docker', 'run', '--rm', '--network', 'host',
            '-v', '/tmp/my_cmd.py:/tmp/my_cmd.py',
            '-e', 'ROS_MASTER_URI=http://192.168.42.2:11311',
            'baxter_ros1', 'bash', '-c',
            'source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /tmp/my_cmd.py'
        ])
        
        print("✅ Movement sent to robot!")
    else:
        print("❌ Planning failed")

if __name__ == '__main__':
    move_baxter()
```

**Run it:**
```bash
docker exec -it baxter_moveit bash -c "cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 my_baxter_script.py"
```

---

## 🔧 Troubleshooting:

### Robot not moving?
```bash
./diagnose_baxter.sh
```

### RViz not showing?
```bash
python3 test_rviz.py
```

### Check what's running:
```bash
docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
```

### Restart everything:
```bash
docker stop baxter_moveit
./start_baxter_moveit.sh
```

---

## 🎯 Current Status Summary:

✅ **MoveIt Planning**: Working  
✅ **Real Robot Communication**: Working  
✅ **Joint Space Control**: Working  
✅ **RViz Visualization**: Working  
✅ **Development Environment**: Ready  

**You can now plan movements in MoveIt and execute them on the real Baxter robot!**

---

## 📝 Quick Daily Workflow:

1. **Start**: `./start_baxter_moveit.sh` (already done)
2. **Test**: `./test_baxter_movement.sh` 
3. **Plan**: Use RViz or write Python scripts
4. **Execute**: Scripts automatically send to real robot
5. **Develop**: `docker exec -it baxter_moveit bash`

**Your system is ready for ROS 2 + MoveIt development with real Baxter! 🤖**