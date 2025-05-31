from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    example_pkg_path = FindPackageShare('robot_pkg')  # Replace with your own package name
    gz_launch_path = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    return LaunchDescription([
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            PathJoinSubstitution([example_pkg_path, 'gazebo_models'])
        ),
        SetEnvironmentVariable(
            'GZ_SIM_PLUGIN_PATH',
            PathJoinSubstitution([example_pkg_path, 'gazebo_plugins'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch_path),
            launch_arguments={
                'gz_args': [PathJoinSubstitution([example_pkg_path, 'gazebo_worlds/building_robot.sdf'])],  # Replace with your own world file
                'on_exit_shutdown': 'True'
            }.items(),
        ),

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU',
                '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                'geometry_msgs/msg/Twist@gz.msgs.Twist'
            ],
            remappings=[('/example_imu_topic',
                         '/remapped_imu_topic'),],
            output='screen'
        ),

        # Launch ROS files
        Node(
            package='robot_pkg',
            executable='test',
            output='screen'
        )
    ])