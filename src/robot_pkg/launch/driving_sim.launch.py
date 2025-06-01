import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_ros_gz_sim     = get_package_share_directory('ros_gz_sim')
    pkg_robot          = get_package_share_directory('robot_pkg')

    # Ignition Gazebo launcher in ros_gz_sim:
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    world_sdf = PathJoinSubstitution([pkg_robot, 'gazebo_worlds', 'building_robot.sdf']) #sdf file path

    set_gz_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ''
    )
    set_gz_plugin = SetEnvironmentVariable(
        'GZ_SIM_PLUGIN_PATH',
        ''
    )

    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': [world_sdf],
            'on_exit_shutdown': 'True'
        }.items()
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge_lidar_cmdvel',
        output='screen',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ]
    )

    drive_node = Node(
        package='robot_pkg',
        executable='drive_action',
        name='drive_straight_until_wall',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(start_gazebo)
    ld.add_action(bridge_node)
    ld.add_action(drive_node)

    return ld