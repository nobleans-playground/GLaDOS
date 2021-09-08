#!/usr/bin/env python

# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots TurtleBot3 Burger driver."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler, EmitEvent, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

from webots_ros2_core.utils import ControllerLauncher
from webots_ros2_core.webots_launcher import WebotsLauncher

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    NAMESPACE = 'glados'
    package_dir = get_package_share_directory('glados_simulation')

    package = LaunchConfiguration('package')
    executable = LaunchConfiguration('executable')
    node_parameters = LaunchConfiguration('node_parameters')
    synchronization = LaunchConfiguration('synchronization')
    publish_tf = LaunchConfiguration('publish_tf')
    robot_name = LaunchConfiguration('robot_name')
    node_name = LaunchConfiguration('node_name')

    world = LaunchConfiguration('world')

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('webots_ros2_core'), 'launch', 'robot_launch.py')
        ),
        launch_arguments=[
            ('package', 'glados_simulation'),
            ('executable', 'turtlebot_driver.py'),
            ('world', PathJoinSubstitution([package_dir, 'worlds', world])),
        ]
    )

    # Driver node
    controller = ControllerLauncher(
        package=package,
        executable=executable,
        parameters=[
            node_parameters,
            {
                'synchronization': synchronization,
                'use_joint_state_publisher': publish_tf
            }],
        output='screen',
        arguments=[
            '--webots-robot-name', robot_name,
            '--webots-node-name', node_name
        ],
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>',
            'use_sim_time': use_sim_time
        }],
        condition=launch.conditions.IfCondition(publish_tf)
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_burger_example.wbt',
            description='Choose one of the world files from `/glados_simulation/worlds` directory'
        ),
        webots,
        controller,
        robot_state_publisher,

        # Shutdown launch when Webots exits.
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
