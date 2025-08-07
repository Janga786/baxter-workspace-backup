from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_setup_assistant',
            executable='moveit_setup_assistant',
            name='moveit_setup_assistant',
            output='screen',
            arguments=[
                '--urdf-path',
                '/home/janga/baxter_bridge_project/shared_ws/urdf/baxter_simple.urdf'
            ]
        )
    ])