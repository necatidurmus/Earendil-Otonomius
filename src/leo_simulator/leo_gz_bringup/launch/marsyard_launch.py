
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_leo_gz_bringup = get_package_share_directory('leo_gz_bringup')
    pkg_leo_gz_worlds = get_package_share_directory('leo_gz_worlds')

    sim_world_arg = DeclareLaunchArgument(
        'sim_world',
        default_value=os.path.join(pkg_leo_gz_worlds, 'worlds', 'marsyard2020.sdf'),
        description='Path to the Gazebo world file'
    )

    leo_gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_leo_gz_bringup, 'launch', 'leo_gz.launch.py')
        ),
        launch_arguments={
            'sim_world': LaunchConfiguration('sim_world')
        }.items()
    )

    return LaunchDescription([
        sim_world_arg,
        leo_gz_launch
    ])
