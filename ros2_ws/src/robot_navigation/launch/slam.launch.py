"""
slam.launch.py
--------------
Launches full simulation + SLAM Toolbox for map building.

Usage:
  ros2 launch robot_navigation slam.launch.py

After mapping, save the map:
  ros2 run nav2_map_server map_saver_cli -f ~/map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav     = get_package_share_directory('robot_navigation')
    pkg_bringup = get_package_share_directory('robot_bringup')

    slam_params = os.path.join(pkg_nav, 'config', 'slam_params.yaml')
    rviz_file   = os.path.join(pkg_nav, 'rviz', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # Gazebo + RSP + spawn (no RViz inside)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'simulation.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # SLAM Toolbox — async mapping mode
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {'use_sim_time': use_sim_time},
            ],
        ),

        # RViz with Nav2/map config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
