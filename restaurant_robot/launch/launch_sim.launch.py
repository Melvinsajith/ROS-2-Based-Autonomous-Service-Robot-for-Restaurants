import os
from launch import LaunchDescription
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'restaurant_robot'  # <--- your package name

    # Include robot_state_publisher launch with use_sim_time enabled
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'robot_pu.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    # Include Gazebo launch with extra args (param file)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file
        }.items()
    )

    gazebo_s = Node(
    package='gazebo_ros',
    executable='gzserver',
    output='screen',
    arguments=['-s', 'libgazebo_ros_factory.so']
    
    )  

    # Spawn robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'restaurant_robot'],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Twist mux node with sim time enabled
    twist_mux_param_file = os.path.join(get_package_share_directory(package_name), 'config', "twist_mux.yaml")

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[
            twist_mux_param_file,
            {"use_sim_time": True}
        ],
        remappings=[
            ("/cmd_vel_out", "/diff_cont/cmd_vel_unstamped")
        ]
    )

    rviz_config_file = os.path.join(
    get_package_share_directory(package_name),
    'config',
    'main.rviz'
)

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )


    # Diff drive controller spawner
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        parameters=[{'use_sim_time': True}]
    )

    # Joint state broadcaster spawner
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        parameters=[{'use_sim_time': True}]
    )

    # Launch all components
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        twist_mux,
        # diff_drive_spawner,
        # joint_broad_spawner,
        # rviz_node,
    ])
