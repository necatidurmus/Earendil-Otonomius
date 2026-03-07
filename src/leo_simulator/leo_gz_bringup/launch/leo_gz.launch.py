# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory("leo_gz_bringup")
    pkg_project_worlds = get_package_share_directory("leo_gz_worlds")

    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(pkg_project_worlds, "worlds", "leo_empty.sdf"),
        description="Path to the Gazebo world file",
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    # Ignition Fortress dünyası adı — clock bridge için gerekli
    # Fortress: /clock değil /world/<name>/clock yayınlar
    world_name = DeclareLaunchArgument(
        "world_name",
        default_value="leo_empty",
        description="Gazebo world name (SDF <world name=...>), used for clock bridge",
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={"robot_ns": LaunchConfiguration("robot_ns")}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    # Ignition Fortress clock topic: /world/<world_name>/clock
    # Remap ile ROS tarafında /clock olarak yayınlanır
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            ["/world/", LaunchConfiguration("world_name"),
             "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        ],
        remappings=[
            (["/world/", LaunchConfiguration("world_name"), "/clock"], "/clock"),
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            sim_world,
            robot_ns,
            world_name,
            gz_sim,
            topic_bridge,
            # Robot spawn 8 saniye geciktiriliyor: Gazebo'nun tamamen
            # başlaması ve entity service'in hazır olması için
            TimerAction(period=8.0, actions=[spawn_robot]),
        ]
    )
