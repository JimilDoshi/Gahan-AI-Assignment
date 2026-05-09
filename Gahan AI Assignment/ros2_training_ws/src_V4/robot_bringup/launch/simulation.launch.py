"""
simulation.launch.py
--------------------
Master simulation launch: Gazebo (server + client) + robot_state_publisher + spawn.
No RViz — include this from slam/navigation launches and add RViz there.

Usage:
  ros2 launch robot_bringup simulation.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup     = get_package_share_directory('robot_bringup')
    pkg_description = get_package_share_directory('robot_description')
    pkg_gazebo_ros  = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(pkg_bringup, 'worlds', 'training_world.world')
    urdf_file  = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')

    robot_description = Command(['xacro ', urdf_file])
    use_sim_time      = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # Gazebo client (GUI)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # Robot State Publisher — broadcasts /robot_description + TF
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

        # Joint State Publisher — publishes non-Gazebo-driven joint states (casters)
        # Gazebo drives wheel joints via diff_drive plugin; caster joints need this.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Spawn robot entity into Gazebo.
        # z=0.0 is correct — base_footprint is the ground frame and the
        # chassis geometry is raised internally via visual/collision origins.
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
                '-z', '0.0',
            ],
        ),
    ])
