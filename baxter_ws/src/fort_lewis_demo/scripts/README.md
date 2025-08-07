# Baxter Master Demonstration Suite

A comprehensive demonstration system for the Baxter robot showcasing interactive storytelling, precise movements, facial expressions, and audience engagement.

## Overview

This demonstration suite integrates multiple Baxter capabilities into a seamless, engaging presentation perfect for educational demonstrations, robotics showcases, and public outreach events.

## Features

### 🎭 Interactive Storytelling
- **Duck Quest**: An engaging narrative where Baxter searches for his lost rubber duck
- Computer vision integration for object detection
- Audience participation and interaction

### 🤖 Technical Demonstrations  
- Precise bilateral arm coordination
- Safety monitoring and collision detection
- Smooth trajectory planning and execution
- Error recovery and graceful degradation

### 😊 Expressive Communication
- Digital facial expressions with geometric aesthetics
- Text-to-speech integration
- Dynamic status displays
- Multi-modal communication (visual + audio)

### 🎯 Audience Interaction
- High-five demonstrations
- Head tracking and acknowledgment
- Real-time audience engagement
- Interactive decision making

## File Structure

```
scripts/
├── master_demo.py              # Main orchestration script
├── baxter_movements.py         # Movement and gesture library
├── baxter_communication.py     # Display and speech systems
├── baxter_face_expressions.py  # Digital expression generator
├── baxter_demo_main.py         # Duck Quest interactive story
├── baxter_demo_setup.py        # Robot setup and calibration
├── baxter_demo_utils.py        # Utility functions and monitoring
└── README.md                   # This documentation
```

## Quick Start

### Prerequisites
- ROS Melodic or later
- Baxter SDK properly installed
- Robot enabled and calibrated
- Clear workspace around the robot

### Running the Demo

1. **Setup and Calibration** (recommended first run):
   ```bash
   rosrun fort_lewis_demo baxter_demo_setup.py
   ```

2. **Run Complete Master Demo**:
   ```bash
   rosrun fort_lewis_demo master_demo.py
   ```

3. **Individual Components** (for testing):
   ```bash
   # Duck Quest story only
   rosrun fort_lewis_demo baxter_demo_main.py
   
   # Expression testing
   rosrun fort_lewis_demo baxter_face_expressions.py
   ```

## Demo Phases

### Phase 1: Introduction (3-4 minutes)
- Wake-up sequence with stretching
- Greeting and audience acknowledgment  
- Introductory wave gesture
- System status overview

### Phase 2: Duck Quest Story (8-10 minutes)
- Interactive narrative presentation
- Computer vision duck detection
- Audience participation (duck placement)
- Celebration sequence upon completion

### Phase 3: Technical Skills (4-5 minutes)
- Precision movement demonstration
- Bilateral arm coordination
- Facial expression showcase
- Safety system overview

### Phase 4: Audience Interaction (3-4 minutes)
- High-five invitation and execution
- Head nodding acknowledgment
- Celebration dance
- Personal interaction moments

### Phase 5: Conclusion (2-3 minutes)
- Thank you sequence
- Final bow gesture
- Return to safe position
- Performance summary display

**Total Duration: ~20-25 minutes**

## Safety Features

### Collision Detection
- Real-time joint torque monitoring
- Automatic motion stopping on high forces
- Recovery procedures for stuck joints

### Velocity Limits
- Joint speed monitoring and limiting
- Smooth acceleration/deceleration profiles
- Emergency stop capabilities

### Error Recovery
- Graceful degradation on component failure
- Automatic retry mechanisms
- Safe position fallback procedures

### Manual Override
- Keyboard interrupt handling (Ctrl+C)
- Emergency stop activation
- Component-level testing modes

## Customization Options

### Timing Adjustments
Edit timing parameters in the respective modules:
- `duration` parameters in communication functions
- `sleep` delays between movements
- Phase transition intervals

### Expression Modifications
Modify `baxter_face_expressions.py` to customize:
- Color schemes and visual aesthetics
- New expression types
- Text display formatting

### Movement Variations
Edit `baxter_movements.py` to adjust:
- Gesture amplitudes and speeds
- New movement sequences
- Position accuracy and smoothness

### Story Content
Customize `baxter_demo_main.py` for:
- Alternative narrative content
- Different interaction scenarios
- Modified object detection parameters

## Troubleshooting

### Common Issues

**Robot won't enable:**
- Check emergency stop status
- Verify network connection
- Run setup script first: `baxter_demo_setup.py`

**Jerky movements:**
- Reduce speed parameters in movement functions
- Check joint calibration status
- Verify workspace is clear

**No facial expressions:**
- Check display topic `/robot/xdisplay`
- Verify cv_bridge installation
- Test with `baxter_face_expressions.py`

**Audio not working:**
- Check speech synthesis topic `/robot/say` 
- Verify audio system configuration
- Test with simple speech commands

### Debug Mode

Run components individually for debugging:

```bash
# Test movements only
python -c "from baxter_movements import BaxterMovements; bm = BaxterMovements(); bm.wave_gesture()"

# Test expressions only
python baxter_face_expressions.py

# Test communication only
python -c "from baxter_communication import BaxterCommunication; bc = BaxterCommunication(); bc.say_and_display('Test message', 'happy')"
```

### Log Analysis

Monitor ROS logs for detailed debugging:
```bash
# View all demo logs
rostopic echo /rosout | grep -i baxter

# Monitor specific error types
rostopic echo /rosout_agg | grep -i error
```

## Performance Optimization

### Timing Optimization
- Adjust `duration` parameters based on audience size
- Modify transition delays for pacing
- Customize interaction timeouts

### Resource Management
- Monitor CPU usage during computer vision phases
- Adjust camera resolution if needed
- Optimize trajectory computation frequency

### Reliability Improvements  
- Increase error handling timeout values
- Add redundant fallback behaviors
- Implement more robust recovery sequences

## Educational Extensions

### Curriculum Integration
- **Robotics Courses**: Demonstrate forward/inverse kinematics
- **Computer Science**: Show computer vision and AI concepts  
- **Engineering**: Explain control systems and safety design
- **Liberal Arts**: Discuss human-robot interaction and ethics

### Interactive Learning
- Pause demo for Q&A sessions
- Explain technical concepts during execution
- Demonstrate programming concepts live
- Compare robot vs human capabilities

### Research Opportunities
- Study audience engagement metrics
- Analyze interaction success rates
- Measure learning outcome improvements
- Evaluate accessibility features

## Maintenance

### Regular Checks
- Verify joint calibration weekly
- Test safety systems before each demo
- Update software dependencies monthly
- Clean camera lenses regularly

### Hardware Maintenance
- Check gripper calibration before use
- Inspect cables and connections
- Monitor joint wear and performance
- Keep workspace clean and organized

### Software Updates
- Monitor ROS package updates
- Test compatibility with new versions
- Backup working configurations
- Document any customizations made

## Support and Contribution

### Getting Help
For technical support or questions:
- Check ROS/Baxter documentation first
- Review log files for specific error messages
- Test individual components in isolation
- Contact robotics team for Fort Lewis specific issues

### Contributing Improvements
We welcome contributions:
- Bug fixes and error handling improvements
- New movement sequences or expressions
- Educational content enhancements
- Performance optimizations

### Feedback
Please provide feedback on:
- Demo effectiveness and engagement
- Technical reliability and robustness
- Audience reaction and learning outcomes
- Suggestions for new features

---

**Created by**: Fort Lewis College Robotics Team  
**Last Updated**: 2024  
**Compatible with**: ROS Melodic, Baxter SDK, Python 2.7  
**License**: Educational Use