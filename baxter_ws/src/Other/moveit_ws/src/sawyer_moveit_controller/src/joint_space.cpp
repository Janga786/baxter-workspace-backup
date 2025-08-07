#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "sawyer_joint_space_control");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup MoveGroupInterface for the "right_arm"
  static const std::string PLANNING_GROUP = "right_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  ROS_INFO("MoveIt C++ control node for joint-space goal started.");

  // Define a complete target joint configuration in radians.
  // The order is j0, j1, j2, j3, j4, j5, j6.
  std::vector<double> joint_group_positions = {
      0.5,    // right_j0 - Base rotation
     -0.7,    // right_j1 - Shoulder
      0.0,    // right_j2 - Elbow link
      1.5,    // right_j3 - Elbow joint
      0.0,    // right_j4 - Wrist link
     -0.8,    // right_j5 - Wrist joint
      0.0     // right_j6 - Gripper rotation
  };
  
  // Set the defined joint values as the motion target.
  move_group_interface.setJointValueTarget(joint_group_positions);

  // Set the speed and acceleration scaling factors. 0.4 means 40% of max.
  move_group_interface.setMaxVelocityScalingFactor(0.4);
  move_group_interface.setMaxAccelerationScalingFactor(0.4);

  // Plan and execute the motion.
  ROS_INFO("Planning and moving to the joint-space goal...");
  bool success = (move_group_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

  // Report the result
  if (success)
  {
    ROS_INFO("Motion to joint-space goal executed successfully!");
  }
  else
  {
    ROS_ERROR("Motion to joint-space goal failed!");
  }

  ros::shutdown();
  return 0;
}
