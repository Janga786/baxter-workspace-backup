#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "sawyer_move_to_pose");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to gecannot open source file "geometry_msgs/Pose.h"C/C++(1696)t information
  // about the robot's state. We run it in a separate thread.
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // The :move_group_interface:`MoveGroupInterface` class is the main tool for controlling the robot.
  // We specify the name of the planning group we want to control. For Sawyer, this is "right_arm".
  static const std::string PLANNING_GROUP = "right_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  ROS_INFO("MoveIt C++ control node started.");

  // 1. Define a target pose
  //    This is a position (x, y, z) and orientation (x, y, z, w) for the gripper.
  //    These coordinates are relative to the robot's 'base' frame.
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0; // A simple orientation: pointing straight down
  target_pose.position.x = 0.6;
  target_pose.position.y = 0.2;
  target_pose.position.z = 0.4;
  
  // Tell the move_group the target pose
  move_group_interface.setPoseTarget(target_pose);

  // Set the speed to 80% of the maximum
  ROS_INFO("Setting velocity and acceleration scaling factors.");
  move_group_interface.setMaxVelocityScalingFactor(0.8);
  move_group_interface.setMaxAccelerationScalingFactor(0.8);

  // 2. Plan the motion
  ROS_INFO("Planning motion to the target pose...");
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // The `move()` command plans and executes the motion.
  bool success = (move_group_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

  // 3. Report the result
  if (success)
  {
    ROS_INFO("Motion executed successfully!");
  }
  else
  {
    ROS_ERROR("Motion failed!");
  }

  // Shut down ROS
  ros::shutdown();
  return 0;
}
