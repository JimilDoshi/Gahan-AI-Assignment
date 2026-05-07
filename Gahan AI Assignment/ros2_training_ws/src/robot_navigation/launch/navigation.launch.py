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
    pkg_nav2    = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    rviz_file   = os.path.join(pkg_nav, 'rviz', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml     = LaunchConfiguration('map', default='')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Full path to map yaml file'
        ),

        # Bring up Gazebo + robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # Nav2 bringup (includes AMCL, planners, controllers, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params,
                'map': map_yaml,
            }.items(),
        ),

        # RViz with Nav2 config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
