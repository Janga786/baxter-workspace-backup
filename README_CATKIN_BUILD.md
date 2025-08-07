# Baxter Station - Catkin Build Setup

This repository contains a complete Baxter robot development workspace configured for `catkin_make` builds.

## Quick Start

### 1. Build the Workspace
```bash
# Build all Baxter packages (excludes bridge projects)
./build_baxter_workspace.sh
```

### 2. Setup Environment  
```bash
# Setup Baxter development environment
source ./setup_baxter_environment.sh
```

### 3. Configure Robot Connection
Edit `setup_baxter_environment.sh` and update the robot hostname:
```bash
export BAXTER_HOSTNAME="192.168.1.100"  # Your robot's IP
```

### 4. Enable Robot and Run Examples
```bash
cd baxter_ws
./baxter.sh            # Enable robot connection
source devel/setup.bash

# Run example programs
rosrun baxter_examples joint_position_keyboard.py
rosrun baxter_examples ik_service_client.py  
rosrun baxter_tools enable_robot.py -e
```

## Package Overview

### Core Packages Built by catkin_make:

- **baxter_common**: Core message definitions and robot description
  - `baxter_core_msgs`: ROS messages for Baxter robot
  - `baxter_description`: URDF files and 3D meshes
  - `baxter_maintenance_msgs`: Maintenance and calibration messages

- **baxter_interface**: Python API for robot control
  - Robot limb control interfaces
  - Gripper, head, and sensor interfaces  
  - Camera and I/O control

- **baxter_examples**: Example scripts and demonstrations
  - Joint position control examples
  - Gripper control demos
  - IK (Inverse Kinematics) examples
  - Trajectory playback utilities

- **baxter_tools**: Robot maintenance and setup utilities  
  - Robot enablement tools
  - Calibration utilities
  - Diagnostic scripts

- **fort_lewis_demo**: Demonstration package
  - Complete robot demonstration scripts

### Excluded Packages:
- Bridge projects (ROS1/ROS2 bridges) - excluded as requested
- Computer vision projects  
- Large development environments

## Build System Details

### Catkin Workspace Structure:
```
baxter_ws/
├── src/
│   ├── CMakeLists.txt -> /opt/ros/indigo/share/catkin/cmake/toplevel.cmake
│   ├── baxter_common/     # Message definitions and description
│   ├── baxter_interface/  # Python robot control API
│   ├── baxter_examples/   # Example scripts
│   ├── baxter_tools/      # Utilities and tools
│   └── fort_lewis_demo/   # Demo package
├── build/                 # Build artifacts (generated)
├── devel/                 # Development space (generated)
└── baxter.sh             # Baxter SDK environment setup
```

### Dependencies:
- ROS Indigo (recommended) or Noetic
- Python 2.7 (for Indigo) or Python 3 (for Noetic)
- Standard catkin build tools

## Manual Build Process

If you prefer to build manually:

```bash
cd baxter_ws

# Source ROS environment
source /opt/ros/indigo/setup.bash  # or noetic

# Clean build (optional)
rm -rf build devel

# Build with catkin_make
catkin_make

# Source the workspace
source devel/setup.bash
```

## Robot Connection Setup

### Network Configuration:
1. Connect to same network as Baxter robot
2. Update `BAXTER_HOSTNAME` in setup script
3. Test connection: `ping your-robot-ip`

### Enable Robot:
```bash
cd baxter_ws
./baxter.sh  # This sources the Baxter SDK environment
rosrun baxter_tools enable_robot.py -e
```

## Troubleshooting

### Build Issues:
- Ensure ROS is properly installed and sourced
- Check that all dependencies are installed: `rosdep install --from-paths src --ignore-src -r -y`
- Clean build if needed: `rm -rf build devel && catkin_make`

### Robot Connection Issues:
- Verify robot IP address and network connectivity
- Check that robot is powered on and network enabled
- Ensure `baxter.sh` environment setup is working

### Package Import Issues:
- Make sure workspace is sourced: `source devel/setup.bash`
- Check Python path includes workspace: `echo $PYTHONPATH`

## Example Usage

### Basic Joint Control:
```python
import rospy
import baxter_interface

rospy.init_node('test_node')
left_arm = baxter_interface.Limb('left')
angles = left_arm.joint_angles()
print("Current joint angles:", angles)
```

### Run Built-in Examples:
```bash
# Keyboard joint control
rosrun baxter_examples joint_position_keyboard.py

# Gripper control demo  
rosrun baxter_examples gripper_keyboard.py

# Head control demo
rosrun baxter_examples head_wobbler.py
```

## Additional Resources

- [Baxter SDK Documentation](http://sdk.rethinkrobotics.com/)
- [ROS Catkin Documentation](http://wiki.ros.org/catkin)
- Original workspace includes additional tools in `baxter_tools/` and examples in `baxter_examples/`