#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project = get_package_share_directory('leo_gz_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_spoofer = LaunchConfiguration('use_spoofer')

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project, 'launch', 'navigation_hybrid.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': launch_rviz,
            'use_spoofer': use_spoofer,
        }.items(),
    )

    goal_bridge = Node(
        package='leo_gz_bringup',
        executable='goal_pose_bridge.py',
        name='goal_pose_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('use_spoofer', default_value='true'),
        base_launch,
        goal_bridge,
    ])
