#include <ros/ros.h>
#include <cmath>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <sensor_msgs/JointState.h>

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

// Global variable to track if we've received joint states
bool joint_states_received = false;
sensor_msgs::JointState latest_joint_state;

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    latest_joint_state = *msg;
    joint_states_received = true;
}

bool waitForRobotState(moveit::planning_interface::MoveGroupInterface& move_group, double timeout = 10.0)
{
    ROS_INFO("Waiting for robot state...");
    ros::Time start_time = ros::Time::now();
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < timeout)
    {
        if (joint_states_received)
        {
            // Try to get current state
            moveit::core::RobotStatePtr current_state = move_group.getCurrentState(1.0);
            if (current_state)
            {
                ROS_INFO("Successfully received robot state!");
                return true;
            }
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    ROS_ERROR("Failed to receive robot state within timeout period");
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sawyer_moveit_demo_robust");
    ros::NodeHandle node_handle;

    // Start async spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Subscribe to joint states to monitor connectivity
    ros::Subscriber joint_state_sub = node_handle.subscribe("/joint_states", 1, jointStateCallback);

    static const std::string PLANNING_GROUP = "right_arm";

    ROS_INFO("Initializing MoveGroupInterface for planning group: %s", PLANNING_GROUP.c_str());

    // Initialize move group interface with error handling
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    try
    {
        move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Failed to initialize MoveGroupInterface: %s", e.what());
        return -1;
    }

    // Wait for robot state before proceeding
    if (!waitForRobotState(*move_group_interface))
    {
        ROS_ERROR("Cannot proceed without valid robot state. Check if:");
        ROS_ERROR("1. Robot is powered on and publishing joint states");
        ROS_ERROR("2. Joint state publisher is running");
        ROS_ERROR("3. Clock synchronization between machines");
        return -1;
    }

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Get joint model group with error checking
    const moveit::core::JointModelGroup* joint_model_group = nullptr;
    try
    {
        joint_model_group = move_group_interface->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
        if (!joint_model_group)
        {
            ROS_ERROR("Failed to get joint model group for: %s", PLANNING_GROUP.c_str());
            return -1;
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception getting joint model group: %s", e.what());
        return -1;
    }

    // Initialize visual tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "Sawyer MoveIt Demo - Robust Version", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Print debug information
    ROS_INFO_NAMED("demo", "Planning frame: %s", move_group_interface->getPlanningFrame().c_str());
    ROS_INFO_NAMED("demo", "End effector link: %s", move_group_interface->getEndEffectorLink().c_str());
    ROS_INFO_NAMED("demo", "Available Planning Groups:");
    
    auto group_names = move_group_interface->getJointModelGroupNames();
    for (const auto& name : group_names)
    {
        ROS_INFO_NAMED("demo", "  - %s", name.c_str());
    }

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    // Demo 1: Simple pose target
    ROS_INFO("=== Demo 1: Simple Pose Target ===");
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.2;
    target_pose1.position.z = 0.5;
    
    move_group_interface->setPoseTarget(target_pose1);
    move_group_interface->setMaxVelocityScalingFactor(0.1);
    move_group_interface->setMaxAccelerationScalingFactor(0.1);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    ROS_INFO_NAMED("demo", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");

    if (success)
    {
        visual_tools.publishAxisLabeled(target_pose1, "pose1");
        visual_tools.publishText(text_pose, "Pose Goal - SUCCESS", rvt::GREEN, rvt::XLARGE);
        visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    }
    else
    {
        visual_tools.publishText(text_pose, "Pose Goal - FAILED", rvt::RED, rvt::XLARGE);
    }
    
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to continue to joint space planning");

    // Demo 2: Joint space planning with safer values
    ROS_INFO("=== Demo 2: Joint Space Planning ===");
    
    moveit::core::RobotStatePtr current_state;
    try
    {
        current_state = move_group_interface->getCurrentState(2.0);
        if (!current_state)
        {
            ROS_ERROR("Failed to get current state for joint space planning");
            visual_tools.publishText(text_pose, "Joint Planning - FAILED (No State)", rvt::RED, rvt::XLARGE);
            visual_tools.trigger();
        }
        else
        {
            std::vector<double> joint_group_positions;
            current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

            // Print current joint positions for debugging
            ROS_INFO("Current joint positions:");
            for (size_t i = 0; i < joint_group_positions.size(); ++i)
            {
                ROS_INFO("  Joint %zu: %f", i, joint_group_positions[i]);
            }

            // Make a small, safe change to the first joint
            if (!joint_group_positions.empty())
            {
                joint_group_positions[0] += 0.2; // Small 0.2 radian change instead of -tau/6
                move_group_interface->setJointValueTarget(joint_group_positions);

                success = (move_group_interface->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                ROS_INFO_NAMED("demo", "Visualizing plan 2 (joint space goal) %s", success ? "SUCCESS" : "FAILED");

                visual_tools.deleteAllMarkers();
                if (success)
                {
                    visual_tools.publishText(text_pose, "Joint Space Goal - SUCCESS", rvt::GREEN, rvt::XLARGE);
                    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
                }
                else
                {
                    visual_tools.publishText(text_pose, "Joint Space Goal - FAILED", rvt::RED, rvt::XLARGE);
                }
            }
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception in joint space planning: %s", e.what());
        visual_tools.publishText(text_pose, "Joint Planning - EXCEPTION", rvt::RED, rvt::XLARGE);
    }
    
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to continue or 'q' to quit");

    // Demo 3: Simple Cartesian path
    ROS_INFO("=== Demo 3: Cartesian Path ===");
    
    try
    {
        std::vector<geometry_msgs::Pose> waypoints;
        
        // Start from current pose
        geometry_msgs::Pose start_pose = move_group_interface->getCurrentPose().pose;
        waypoints.push_back(start_pose);

        // Simple up movement
        geometry_msgs::Pose target_pose = start_pose;
        target_pose.position.z += 0.1;
        waypoints.push_back(target_pose);

        // Simple right movement
        target_pose.position.y += 0.1;
        waypoints.push_back(target_pose);

        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01;
        const double jump_threshold = 0.0; // Disable jump threshold
        
        double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        ROS_INFO_NAMED("demo", "Cartesian path (%.2f%% achieved)", fraction * 100.0);

        visual_tools.deleteAllMarkers();
        visual_tools.publishText(text_pose, "Cartesian Path", rvt::WHITE, rvt::XLARGE);
        visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
        
        if (fraction > 0.0)
        {
            visual_tools.publishTrajectoryLine(trajectory, joint_model_group);
            for (std::size_t i = 0; i < waypoints.size(); ++i)
                visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Exception in Cartesian path planning: %s", e.what());
    }
    
    visual_tools.trigger();
    visual_tools.prompt("Demo complete! Press 'next' to exit");

    ROS_INFO("Demo completed successfully!");
    ros::shutdown();
    return 0;
}
