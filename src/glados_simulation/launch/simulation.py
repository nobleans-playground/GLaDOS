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

"""Launch Webots GLaDOS Simulation."""

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

package_path = get_package_share_directory('glados_simulation')

ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='glados',
        description=''
    ),
    DeclareLaunchArgument(
        'synchronization',
        default_value='False',
        description='If `False` robot.step() will be automatically called.'
    ),
    DeclareLaunchArgument(
        'package',
        default_value='glados_simulation',
        description='The Package in which the node executable can be found.'
    ),
    DeclareLaunchArgument(
        'executable',
        default_value='glados_driver.py',
        description='The name of the executable to find if a package is provided or otherwise a path to the executable to run.'
    ),
    DeclareLaunchArgument(
        'world',
        default_value=os.path.join(package_path, 'worlds', 'turtlebot3_burger_example.wbt'),
        description='Choose one of the world files from `/glados_simulation/worlds` directory'
    ),
    DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Whether to start GUI or not.'
    ),
    DeclareLaunchArgument(
        'mode',
        default_value='realtime',
        description='Choose the startup mode (it must be either `pause`, `realtime`, `run` or `fast`).'
    ),
    DeclareLaunchArgument(
        'publish_tf',
        default_value='True',
        description='Whether to publish transforms (tf)'
    ),
    DeclareLaunchArgument(
        'node_parameters',
        description='Path to ROS parameters file that will be passed to the robot node',
        default_value=os.path.join(package_path, 'param', 'ros2control.yml')
    ),
    DeclareLaunchArgument(
        'robot_name',
        description='The name of the robot (has to be the same as in Webots)',
        default_value=''
    ),
    DeclareLaunchArgument(
        'node_name',
        description='The name of the ROS node that interacts with Webots',
        default_value='webots_driver'
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        description='Whether to use the simulation (Webots) time',
        default_value='True'
    )
]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    package = LaunchConfiguration('package')
    executable = LaunchConfiguration('executable')
    node_parameters = LaunchConfiguration('node_parameters')
    synchronization = LaunchConfiguration('synchronization')
    publish_tf = LaunchConfiguration('publish_tf')
    robot_name = LaunchConfiguration('robot_name')
    node_name = LaunchConfiguration('node_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    mode = LaunchConfiguration('mode')

    # Webots
    webots = WebotsLauncher(
        world=world,
        mode=mode,
        gui=gui
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


    return LaunchDescription(ARGUMENTS + [
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
