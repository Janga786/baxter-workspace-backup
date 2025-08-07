#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <vector>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Pose.h>
#include "shape_msgs/SolidPrimitive.h"

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "sawyer_cartesian_and_obstacles_demo");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  // Setup MoveGroupInterface and PlanningSceneInterface
  static const std::string PLANNING_GROUP = "right_arm";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  ROS_INFO("Cartesian path and collision object demo started.");
  
  // == PHASE 1: Move to a known starting pose ==
  // This pose is known to be away from singularities.
  ROS_INFO("Phase 1: Moving to a starting pose.");
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.6;
  start_pose.position.y = 0.0;
  start_pose.position.z = 0.5;
  move_group_interface.setPoseTarget(start_pose);
  move_group_interface.move();
  
  // A short sleep to ensure the robot has settled
  ros::Duration(1.0).sleep();

  // == PHASE 2: Add a collision object to the world ==
  ROS_INFO("Phase 2: Adding a collision object (a box).");
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface.getPlanningFrame();
  collision_object.id = "box1"; // Give the object a unique name

  // Define the primitive (shape) and its dimensions
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.4;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
  primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.4;

  // Define the pose of the object (where it is in the world)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.6;
  box_pose.position.y = -0.3; // Place it to the side of the robot's path
  box_pose.position.z = 0.3;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Create a vector of objects and add it to the scene
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface.addCollisionObjects(collision_objects);
  
  // A short sleep is needed to ensure the planning scene is updated
  ros::Duration(1.0).sleep();

  // == PHASE 3: Plan a more achievable Cartesian path ==
  ROS_INFO("Phase 3: Planning an achievable Cartesian path (away from the obstacle).");
  
  std::vector<geometry_msgs::Pose> waypoints;
  
  // We use our known 'start_pose' as the beginning of the path.
  geometry_msgs::Pose target_waypoint = start_pose;
  
  // Waypoint 1: Move right by 20cm (away from the obstacle at y=-0.3)
  target_waypoint.position.y += 0.2;
  waypoints.push_back(target_waypoint);

  // Waypoint 2: Move left by 20cm (back to the start pose)
  target_waypoint.position.y -= 0.2;
  waypoints.push_back(target_waypoint);


  // We want to plan this path at a resolution of 1 cm.
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  ROS_INFO("Cartesian path plan (%.2f%% achieved)", fraction * 100.0);

  // Execute the plan if it was fully successful
  if (fraction > 0.99)
  {
    ROS_INFO("Executing Cartesian path...");
    move_group_interface.execute(trajectory);
  }
  else
  {
    ROS_WARN("Could not compute a complete Cartesian path. Robot will not move.");
  }

  // == PHASE 4: Remove the collision object ==
  ROS_INFO("Phase 4: Removing the collision object.");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  ROS_INFO("Demo complete.");
  ros::shutdown();
  return 0;
}

  
  
  
  
  
  
  
