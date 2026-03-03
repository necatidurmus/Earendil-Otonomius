# Copyright 2026 Necati Durmus
# Navigation launch file for Leo Rover with Nav2
# Usage: ros2 launch leo_gz_bringup navigation.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_project_gazebo = get_package_share_directory("leo_gz_bringup")

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )

    declare_map_file = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to map yaml file to load",
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            pkg_project_gazebo, "config", "nav2_params.yaml"
        ),
        description="Full path to the ROS2 parameters file for Nav2",
    )

    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the Nav2 stack",
    )

    # Include Nav2 bringup launch
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": map_file,
            "params_file": nav2_params_file,
            "autostart": autostart,
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_map_file,
            declare_params_file,
            declare_autostart,
            nav2_bringup,
        ]
    )
