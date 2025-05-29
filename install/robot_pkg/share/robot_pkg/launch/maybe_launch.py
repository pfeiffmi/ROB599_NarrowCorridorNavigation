from launch import LaunchDescription
from launch_ros.actions import Node
import pathlib

def generate_launch_description():
    project_ws = pathlib.Path(__file__).parents[5]
    sdf_path = project_ws / "src" / "robot_pkg" / "gazebo_worlds" / "building_robot.sdf"

    return LaunchDescription([
        # Camera Node
        Node(
            package = 'robot_pkg',
            executable = 'test',
            # Launch the node with root access (for video port access) in a shell (from: https://answers.ros.org/question/369846/)
            # commenting out sudo command prefix below as it interferes with communication of nodes on the same machine
            prefix = [f"gz sim {str(sdf_path)} &&"],
            shell = True
        ),
    ])