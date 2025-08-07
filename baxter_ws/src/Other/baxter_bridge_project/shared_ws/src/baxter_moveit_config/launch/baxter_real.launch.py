import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "baxter_ip",
            default_value="192.168.42.2",
            description="IP address of the Baxter robot",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    baxter_ip = LaunchConfiguration("baxter_ip")

    # Include the move_group launch file
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("baxter_moveit_config"),
                "launch",
                "move_group.launch.py"
            )
        ]),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    # Simple Baxter System - provides both joint states and MovEIt action servers
    simple_baxter_system = ExecuteProcess(
        cmd=["python3", "/tmp/simple_baxter_system.py"],
        name="simple_baxter_system",
        output="screen",
    )
    
    # Robot state publisher for transforms
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": open("/shared_ws/urdf/baxter.urdf").read(),
            "use_sim_time": False
        }],
    )
    
    nodes_to_start = [
        simple_baxter_system,
        robot_state_publisher,
        move_group_launch,
    ]

    return nodes_to_start