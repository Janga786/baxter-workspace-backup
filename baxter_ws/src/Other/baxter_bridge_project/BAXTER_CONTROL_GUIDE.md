# 🤖 Baxter Control Guide - Complete MovEIt Integration

## ✅ System Status: READY FOR REAL ROBOT MOVEMENT

Your Baxter ROS2-to-ROS2 bridge with MovEIt integration is **100% working**! 

## 🚀 Quick Start - 2 Scripts to Move Baxter

### Script 1: Launch RViz with MovEIt
```bash
./1_launch_rviz_moveit.sh
```
**What this does:**
- ✅ Launches RViz2 with proper X11 forwarding
- ✅ Loads complete MovEIt configuration
- ✅ Shows Baxter robot model
- ✅ Provides Motion Planning interface
- ✅ Ready for planning and visualization

### Script 2: Program Baxter Movement
```bash
# Copy to container first
docker cp 2_move_baxter.py baxter_moveit:/tmp/2_move_baxter.py

# Run the movement script
docker exec -it baxter_moveit bash -c 'source /opt/ros/humble/setup.bash && python3 /tmp/2_move_baxter.py'
```

**What this does:**
- 🎯 Connects to both arm action servers
- 🎮 Provides demo movements (wave, reach, neutral positions)
- 🕹️ Interactive mode for custom movements
- 🎭 Real-time control of Baxter arms

## 🎮 Movement Options

### Demo Mode
Choose option `1` for automated demo:
1. **Neutral Position** - Safe starting pose
2. **Wave Motion** - Left arm waving
3. **Reach Forward** - Both arms extending
4. **Return to Neutral** - Safe ending pose

### Interactive Mode  
Choose option `2` for manual control:
- `left neutral` - Move left arm to neutral
- `right neutral` - Move right arm to neutral  
- `both neutral` - Move both arms to neutral
- `wave` - Wave with left arm
- `reach` - Reach forward with both arms
- `demo` - Run full demo sequence
- `quit` - Exit program

## 🔧 System Architecture (What's Working)

### ✅ ROS1 to ROS2 Bridge
- **Joint States**: ROS1 `/robot/joint_states` → ROS2 `/joint_states`
- **Commands**: ROS2 action servers → ROS1 robot interface
- **Docker Integration**: Seamless container communication

### ✅ MovEIt Integration
- **Action Servers**: `/left_arm_controller/follow_joint_trajectory` & `/right_arm_controller/follow_joint_trajectory`
- **Planning Groups**: `left_arm`, `right_arm`, `both_arms`
- **Real-time Planning**: OMPL planner with collision detection

### ✅ Verified Components
- ✅ Joint state publishing (15 joints)
- ✅ Action server responsiveness
- ✅ MovEIt move_group running
- ✅ RViz visualization working
- ✅ Trajectory execution ready

## 🎯 Real Robot Movement

### Prerequisites
1. **Robot Enabled**: Use the enable button or:
   ```bash
   ./enable_baxter_simple.sh
   ```

2. **System Running**: Both scripts launched

### Safety Notes
- ⚠️ **Real robot movement**: This system WILL move the physical robot
- ⚠️ **Clear workspace**: Ensure no obstacles around robot
- ⚠️ **Emergency stop**: Keep emergency stop accessible
- ⚠️ **Supervised operation**: Always supervise robot movement

## 🛠️ Advanced Usage

### Custom Joint Positions
Modify positions in `2_move_baxter.py`:
```python
# 7 joint values for each arm: [s0, s1, e0, e1, w0, w1, w2]
custom_position = [0.0, -0.5, 0.0, 1.0, 0.0, 1.5, 0.0]
mover.move_arm('left', custom_position, duration=4.0)
```

### MovEIt Planning via RViz
1. Open RViz (Script 1)
2. Select planning group in Motion Planning panel
3. Drag interactive markers to set goal
4. Click "Plan" button
5. Click "Execute" to move robot

### Programmatic MovEIt Control
Create custom Python scripts using:
```python
from moveit_commander import RobotCommander, PlanningSceneInterface
# Your custom planning code here
```

## 🔍 Troubleshooting

### RViz Won't Open
```bash
# Enable X11 forwarding
xhost +local:docker
export DISPLAY=:0
./1_launch_rviz_moveit.sh
```

### Action Servers Not Found
```bash
# Check if MovEIt is running
docker exec baxter_moveit bash -c 'source /opt/ros/humble/setup.bash && ros2 action list'
```

### Robot Not Moving
1. Check robot enable status
2. Verify joint states are publishing
3. Ensure no E-stop activation

## 📊 System Verification Commands

```bash
# Check joint states
docker exec baxter_moveit bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /joint_states --once'

# Check action servers
docker exec baxter_moveit bash -c 'source /opt/ros/humble/setup.bash && ros2 action list'

# Check MovEIt status
docker exec baxter_moveit bash -c 'source /opt/ros/humble/setup.bash && ros2 node list | grep move_group'
```

## 🎉 Success Confirmation

Your system is working if:
- ✅ RViz opens and shows Baxter model
- ✅ Joint states are updating (15 joints visible)
- ✅ Action servers respond (both arms)
- ✅ MovEIt planning works in RViz
- ✅ Movement script connects successfully

**🚀 Ready for real robot movement!**