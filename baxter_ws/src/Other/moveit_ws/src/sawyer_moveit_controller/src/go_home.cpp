#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "sawyer_go_home");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup MoveGroupInterface
  static const std::string PLANNING_GROUP = "right_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  ROS_INFO("Go Home node started. Moving robot to a defined home position...");

  // --- Manually Define the "Home" Position ---
  // We create a vector of doubles and specify the angle for each of the 7 joints.
  // This is a known, safe, neutral position for the Sawyer.
  std::vector<double> home_joint_positions = {
      0.0,    // right_j0
     -1.18,   // right_j1
      0.0,    // right_j2
      2.18,   // right_j3
      0.0,    // right_j4
      0.57,   // right_j5
      3.316   // right_j6
  };
  
  // Set this vector of joint values as the target for the arm.
  move_group_interface.setJointValueTarget(home_joint_positions);
  
    // Set the speed to 80% of the maximum
  ROS_INFO("Setting velocity and acceleration scaling factors.");
  move_group_interface.setMaxVelocityScalingFactor(0.8);
  move_group_interface.setMaxAccelerationScalingFactor(0.8);
  
  // Plan and execute
  move_group_interface.move();
  
  ROS_INFO("Robot is at home position.");
  ros::shutdown();
  return 0;
}
