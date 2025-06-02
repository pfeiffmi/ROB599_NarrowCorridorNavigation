from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch ROS files
        Node(
            package='interface_pkg',
            executable='gui',
            output='screen'
        ),
    ])