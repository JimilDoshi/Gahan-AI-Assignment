import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_bringup     = get_package_share_directory('robot_bringup')
    pkg_description = get_package_share_directory('robot_description')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')

    # Paths
    world_file = os.path.join(pkg_bringup, 'worlds', 'training_world.world')
    urdf_file  = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    rviz_file  = os.path.join(pkg_description, 'rviz', 'robot.rviz')

    # Robot description via xacro
    robot_description = Command(['xacro ', urdf_file])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # Start Gazebo server with custom world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # Start Gazebo client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),

        # Spawn robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'diff_drive_robot',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
