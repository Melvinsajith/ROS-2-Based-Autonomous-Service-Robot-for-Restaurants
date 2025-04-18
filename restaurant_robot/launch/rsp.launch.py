import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get package path and xacro file
    pkg_path = get_package_share_directory('restaurant_robot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Parse the xacro file
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Use sim time (true in simulation)
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Node to publish robot_state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        robot_state_publisher_node
    ])
