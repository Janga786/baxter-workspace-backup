# 🚀 Baxter ROS 2 MoveIt Daily Quickstart Guide

## 📋 Prerequisites Checklist
- [ ] Baxter robot powered on and calibrated
- [ ] Emergency stop button released  
- [ ] Network connection to 192.168.42.2
- [ ] Docker installed and running

---

## 🎯 Quick Start Commands

### 1. Start Complete MoveIt System (One Command!)
```bash
cd /home/janga/baxter_bridge_project
./start_baxter_moveit.sh
```

### 2. Manual Step-by-Step Launch

#### Terminal 1: Start MoveIt with Visualization
```bash
docker run -d --name baxter_moveit \
  --network host \
  -v /home/janga/baxter_bridge_project/shared_ws:/shared_ws \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --privileged \
  baxter_moveit2

# Launch MoveIt + RViz
docker exec -d baxter_moveit bash -c \
  "cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch baxter_moveit_config demo.launch.py"
```

#### Terminal 2: Test Robot Movement
```bash
# Enable and test Baxter
docker run --rm --network host \
  -v /home/janga/baxter_bridge_project:/scripts \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 bash -c \
  "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /scripts/move_baxter_correct.py"
```

---

## 🎮 Using MoveIt for Path Planning

### In RViz (should open automatically):

1. **Planning Group**: Select "left_arm" or "right_arm"
2. **Goal State**: Drag the orange robot model to desired position
3. **Plan**: Click "Plan" button to generate trajectory
4. **Execute**: Click "Execute" to send to real robot

### Key RViz Panels:
- **Motion Planning**: Main control panel
- **Planning Groups**: Switch between arms
- **Scene Objects**: Add obstacles
- **Displays**: Toggle robot model, planning scene

---

## 💻 Writing Your Own ROS 2 Code

### Template: Simple Movement Script
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import moveit_commander
import geometry_msgs.msg
import sys

class BaxterController(Node):
    def __init__(self):
        super().__init__('baxter_controller')
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Planning groups
        self.left_arm = moveit_commander.MoveGroupCommander("left_arm")
        self.right_arm = moveit_commander.MoveGroupCommander("right_arm")
        
        self.get_logger().info('Baxter controller initialized')
    
    def move_left_arm_to_pose(self, x, y, z, roll, pitch, yaw):
        """Move left arm to specific pose"""
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        # Convert RPY to quaternion (simplified)
        pose_goal.orientation.w = 1.0
        
        self.left_arm.set_pose_target(pose_goal)
        plan = self.left_arm.plan()
        
        if plan[0]:  # If planning succeeded
            self.left_arm.execute(plan[1], wait=True)
            self.get_logger().info('Movement completed')
        else:
            self.get_logger().error('Planning failed')
    
    def move_to_joint_positions(self, joint_positions):
        """Move to specific joint positions"""
        self.left_arm.set_joint_value_target(joint_positions)
        plan = self.left_arm.plan()
        
        if plan[0]:
            self.left_arm.execute(plan[1], wait=True)

def main(args=None):
    rclpy.init(args=args)
    controller = BaxterController()
    
    # Example: Move to a specific pose
    controller.move_left_arm_to_pose(0.5, 0.2, 0.3, 0, 0, 0)
    
    rclpy.spin(controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Run Your Code:
```bash
# Save as my_baxter_script.py, then:
docker exec -it baxter_moveit bash -c \
  "cd /shared_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 my_baxter_script.py"
```

---

## 🔧 Development Environment Setup

### Connect to Development Container:
```bash
docker exec -it baxter_moveit bash
cd /shared_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### Build New Packages:
```bash
# Create new package
ros2 pkg create --build-type ament_python my_baxter_package

# Build workspace
colcon build --packages-select my_baxter_package

# Source new package
source install/setup.bash
```

---

## 🚨 Troubleshooting

### Quick Diagnostic:
```bash
docker run --rm --network host \
  -v /home/janga/baxter_bridge_project:/scripts \
  -e ROS_MASTER_URI=http://192.168.42.2:11311 \
  baxter_ros1 bash -c \
  "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /scripts/simple_baxter_check.py"
```

### Common Issues:
- **No robot movement**: Check emergency stop, run diagnostic
- **RViz won't open**: Check X11 forwarding, DISPLAY variable
- **Planning fails**: Check joint limits, collision detection
- **Connection lost**: Restart containers, check network

### Reset Everything:
```bash
docker stop baxter_moveit
docker rm baxter_moveit
# Then restart with quickstart commands
```

---

## 📚 Useful Commands

### Check System Status:
```bash
# List running nodes
docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 node list"

# Check topics
docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Monitor joint states
docker exec baxter_moveit bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /joint_states"
```

### Move Robot Manually:
```bash
# Wave motion
docker run --rm --network host -v /home/janga/baxter_bridge_project:/scripts -e ROS_MASTER_URI=http://192.168.42.2:11311 baxter_ros1 bash -c "source /opt/ros/indigo/setup.bash && source /ros/ws_baxter/devel/setup.bash && python /scripts/move_baxter_correct.py"
```

---

## 🎯 Daily Workflow

1. **Start**: Run quickstart command
2. **Plan**: Use RViz to plan movements
3. **Code**: Write ROS 2 Python scripts
4. **Test**: Execute movements on real robot
5. **Debug**: Use diagnostic tools if needed
6. **Stop**: `docker stop baxter_moveit` when done

---

## 📁 Important Files

- **MoveIt Config**: `/shared_ws/src/baxter_moveit_config/`
- **Launch Files**: `/shared_ws/src/baxter_moveit_config/launch/`
- **Your Scripts**: `/shared_ws/` (auto-mounted)
- **Diagnostics**: `/home/janga/baxter_bridge_project/simple_baxter_check.py`

---

## 🏆 Success! 
Your Baxter is now ready for ROS 2 development with MoveIt path planning! 

**Next Steps**: Open RViz, drag the robot to a new pose, plan a path, and execute it! 🤖