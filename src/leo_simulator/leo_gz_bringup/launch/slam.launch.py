# Copyright 2026 Necati Durmus
# SLAM launch file for Leo Rover
# Usage: ros2 launch leo_gz_bringup slam.launch.py

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # SLAM Toolbox node
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "solver_plugin": "solver_plugins::CeresSolver",
                "ceres_linear_solver": "SPARSE_NORMAL_CHOLESKY",
                "ceres_preconditioner": "SCHUR_JACOBI",
                "ceres_trust_strategy": "LEVENBERG_MARQUARDT",
                "ceres_dogleg_type": "TRADITIONAL_DOGLEG",
                "ceres_loss_function": "None",
                "odom_frame": "odom",
                "map_frame": "map",
                "base_frame": "base_footprint",
                "scan_topic": "/scan",
                "mode": "mapping",
                "debug_logging": False,
                "throttle_scans": 1,
                "transform_publish_period": 0.02,
                "map_update_interval": 5.0,
                "resolution": 0.05,
                "max_laser_range": 12.0,
                "minimum_time_interval": 0.5,
                "transform_timeout": 0.2,
                "tf_buffer_duration": 30.0,
                "stack_size_to_use": 40000000,
                "enable_interactive_mode": True,
                # Loop closure params
                "use_scan_matching": True,
                "use_scan_barycenter": True,
                "minimum_travel_distance": 0.5,
                "minimum_travel_heading": 0.5,
                "scan_buffer_size": 10,
                "scan_buffer_maximum_scan_distance": 10.0,
                "loop_search_maximum_distance": 3.0,
                "do_loop_closing": True,
                "loop_match_minimum_chain_size": 10,
                "loop_match_maximum_variance_coarse": 3.0,
                "loop_match_minimum_response_coarse": 0.35,
                "loop_match_minimum_response_fine": 0.45,
                # Correlation params
                "correlation_search_space_dimension": 0.5,
                "correlation_search_space_resolution": 0.01,
                "correlation_search_space_smear_deviation": 0.1,
                # Loop closure correlation params
                "loop_search_space_dimension": 8.0,
                "loop_search_space_resolution": 0.05,
                "loop_search_space_smear_deviation": 0.03,
                # Scan matcher params
                "distance_variance_penalty": 0.5,
                "angle_variance_penalty": 1.0,
                "fine_search_angle_offset": 0.00349,
                "coarse_search_angle_offset": 0.349,
                "coarse_angle_resolution": 0.0349,
                "minimum_angle_penalty": 0.9,
                "minimum_distance_penalty": 0.5,
                "use_response_expansion": True,
            }
        ],
        remappings=[("/scan", "/scan")],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            slam_toolbox,
        ]
    )
