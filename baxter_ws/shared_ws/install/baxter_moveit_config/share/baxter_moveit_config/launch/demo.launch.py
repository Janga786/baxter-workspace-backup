import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="moveit.rviz",
            description="RViz configuration file",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    # Planning context
    baxter_moveit_config = FindPackageShare("baxter_moveit_config")
    baxter_description = FindPackageShare("baxter_description")
    
    # Robot description
    robot_description_content = xacro.process_file(
        os.path.join(
            get_package_share_directory("baxter_description"),
            "urdf",
            "baxter.urdf.xacro",
        )
    ).toxml()
    
    robot_description = {"robot_description": robot_description_content}

    # Robot description semantic
    robot_description_semantic_content = open(
        os.path.join(
            get_package_share_directory("baxter_moveit_config"),
            "srdf",
            "baxter.srdf",
        ),
        "r",
    ).read()
    
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Kinematics
    kinematics_yaml = os.path.join(
        get_package_share_directory("baxter_moveit_config"),
        "config",
        "kinematics.yaml",
    )

    # Planning pipeline
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = os.path.join(
        get_package_share_directory("baxter_moveit_config"),
        "config",
        "ompl_planning.yaml",
    )
    
    # Trajectory execution
    moveit_simple_controllers_yaml = os.path.join(
        get_package_share_directory("baxter_moveit_config"),
        "config",
        "moveit_controllers.yaml",
    )
    
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # Joint limits
    joint_limits_yaml = os.path.join(
        get_package_share_directory("baxter_moveit_config"),
        "config",
        "joint_limits.yaml",
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            ompl_planning_yaml,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_parameters,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution(
        [baxter_moveit_config, "rviz", rviz_config]
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    nodes_to_start = [
        static_tf,
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node,
    ]

    return nodes_to_start