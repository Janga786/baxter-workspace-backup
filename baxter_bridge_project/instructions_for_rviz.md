# 🎮 Using RViz to Control Your Baxter Robot

## ✅ RViz is Now Running!

Your RViz window should be open. Here's how to use it to control the physical Baxter robot:

## 🛠️ Initial Setup in RViz:

### 1. **Add Motion Planning Display** (if not present):
   - Click "Add" button at bottom left
   - Select "MotionPlanning" from the list
   - Click "OK"

### 2. **Configure the Display**:
   - In the left panel, expand "MotionPlanning"
   - Set **Fixed Frame** to `base` or `world`
   - Enable "Robot Model" to see Baxter
   - Enable "Planning Scene" to see the environment

### 3. **Motion Planning Panel** (should appear on left):
   - **Planning Group**: Select `left_arm` or `right_arm`
   - **Query Goal State**: Check this box
   - **Show Trail**: Check this to see trajectory

## 🎯 How to Move Baxter:

### **Method 1: Interactive Markers**
1. You should see **orange interactive markers** around the end-effector
2. **Drag these markers** to set target position
3. Click **"Plan"** button to generate trajectory
4. Click **"Execute"** to move the real robot! 🤖

### **Method 2: Random Valid Goals**
1. Click **"Random Valid Goal"** to generate a random target
2. Click **"Plan"** to create trajectory  
3. Click **"Execute"** to move Baxter

### **Method 3: Joint Target**
1. In Planning tab, expand "Joints"
2. Move sliders to set joint angles
3. Click **"Plan"** then **"Execute"**

## ⚠️ Important Notes:

- **Real Robot Movement**: When you click "Execute", the PHYSICAL Baxter will move!
- **Safety**: Make sure workspace is clear before executing
- **Speed**: Movements execute at a safe, controlled speed
- **Stop**: Use emergency stop on Baxter if needed

## 🔧 Troubleshooting:

### If robot model not visible:
- Check "Robot Model" is enabled in displays
- Set Fixed Frame to "base"
- Try "Reset View" in 3D view

### If planning fails:
- Try different goal positions
- Check joint limits aren't exceeded
- Ensure goal is reachable

### If execution doesn't move robot:
- Check that Baxter is enabled
- Verify bridge connections are working
- Check that action servers are running

## 🎉 Success Indicators:

- ✅ Orange robot model visible in 3D view
- ✅ Interactive markers appear on end-effector  
- ✅ "Plan" generates blue trajectory visualization
- ✅ "Execute" moves the physical Baxter robot

## 🚀 Ready to Control Baxter!

Your ROS 2 + MoveIt + RViz system is fully integrated with the physical robot. Any trajectories you plan and execute in RViz will move the real Baxter hardware!

**Happy robot programming!** 🤖✨