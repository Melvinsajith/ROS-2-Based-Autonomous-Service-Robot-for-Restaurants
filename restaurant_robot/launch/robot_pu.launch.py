import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Get package and xacro file path
    pkg_path = get_package_share_directory('restaurant_robot')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Process Xacro file
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # Launch config
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Joint State Publisher GUI (you can switch to headless version if needed)
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Robot State Publisher
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

    # Optional static TF: base_footprint → base_link (only if not in URDF)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        joint_state_publisher_node,
        robot_state_publisher_node,
        # static_tf_node  # remove if your URDF already includes this transform
    ])
