#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/OrientationConstraint.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/Pose.h>
#include <vector>

int main(int argc, char** argv)
{
  // Initialize the ROS node
  ros::init(argc, argv, "sawyer_path_constraints_demo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Setup MoveGroupInterface
  static const std::string PLANNING_GROUP = "right_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  ROS_INFO("MoveIt C++ path constraint node started.");

  // --- SET A VIRTUAL START STATE (ROBUST METHOD) ---
  ROS_INFO("Setting a virtual start state.");

  // Get a smart pointer to the robot's model.
  moveit::core::RobotModelConstPtr robot_model = move_group_interface.getRobotModel();
  
  // Create a RobotState object from the model.
  moveit::core::RobotState start_state(robot_model);
  
  // Get a pointer to the joint model group.
  const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);

  // Define the joint values for our virtual start state (a known "ready" position).
  std::vector<double> joint_start_positions = {
      0.0,    // right_j0
     -1.18,   // right_j1
      0.0,    // right_j2
      2.18,   // right_j3
      0.0,    // right_j4
      0.57,   // right_j5
      3.316   // right_j6
  };

  // Set the joint positions in our 'start_state' object.
  start_state.setJointGroupPositions(joint_model_group, joint_start_positions);
  
  // Tell MoveIt to use this virtual state as the starting point for the next plan.
  move_group_interface.setStartState(start_state);

  // --- DEFINE THE PATH CONSTRAINT ---
  ROS_INFO("Defining path constraints.");
  
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "right_hand";
  ocm.header.frame_id = "base";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.2;
  ocm.absolute_y_axis_tolerance = 0.2;
  ocm.absolute_z_axis_tolerance = 3.14;
  ocm.weight = 1.0;

  // Put the constraint into a `Constraints` message
  moveit_msgs::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);
  
  // Apply the constraint to the planning group
  move_group_interface.setPathConstraints(constraints);
  
  // --- PLAN TO A GOAL ---
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.6;
  target_pose.position.y = -0.3;
  target_pose.position.z = 0.5;
  move_group_interface.setPoseTarget(target_pose);
  
  // --- NEW, MORE ROBUST PLANNING AND EXECUTION ---
  ROS_INFO("Planning motion with path constraints...");
  
  // Increase the planning time to give the solver a better chance on this hard problem.
  move_group_interface.setPlanningTime(15.0);

  // Create a plan object
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  // Attempt to plan the motion.
  bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
      ROS_INFO("Plan found. Executing...");
      // If the plan was successful, execute it.
      move_group_interface.execute(my_plan);
  }
  else
  {
      ROS_ERROR("Failed to find a plan for the constrained motion.");
  }
  
  // --- CLEAR THE CONSTRAINT ---
  ROS_INFO("Clearing path constraints.");
  move_group_interface.clearPathConstraints();

  ros::shutdown();
  return 0;
}

