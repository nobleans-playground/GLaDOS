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
import pathlib
import launch
from launch.substitutions import LaunchConfiguration, Command
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
        default_value=os.path.join(package_path, 'worlds', 'glados.wbt'),
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
        default_value=os.path.join(package_path, 'param', 'diff_drive_control.yml')
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

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    # base_path = os.path.realpath(get_package_share_directory('glados_description')) # also tried without realpath
    # urdf_path = os.path.join(base_path, 'urdf')
    # xacro_file = os.path.join(urdf_path, 'glados.urdf.xacro')

    simulation_dir = get_package_share_directory('glados_simulation')
    webots_description = pathlib.Path(os.path.join(simulation_dir, 'resource', 'glados_webots.urdf')).read_text()
    # webots_description = pathlib.Path(os.path.join(simulation_dir, 'resource', 'robleo_webots.urdf')).read_text()

    diff_drive_control_params = os.path.join(package_path, 'param', 'diff_drive_control.yml')
    # diff_drive_control_params = os.path.join(package_path, 'param', 'ros2control_robleo.yml')

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
        # world=os.path.join(package_path, 'worlds', 'robleo.wbt'),
        mode=mode,
        gui=gui
    )

    controller_manager_timeout = ['--controller-manager-timeout', '50'] if os.name == 'nt' else []
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else "bash -c 'sleep 10; $0 $@' "

# The controller_manager/spawner, spawns controllers and if a manager does not exist yet, it will also be spawned.
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )

    # Driver node
    # controller = ControllerLauncher(
    #     package=package,
    #     executable=executable,
    #     parameters=[
    #         # node_parameters,
    #         {
    #             'synchronization': synchronization,
    #             'use_joint_state_publisher': publish_tf
    #         }],
    #     output='screen',
    #     arguments=[
    #         '--webots-robot-name', robot_name,
    #         '--webots-node-name', node_name
    #     ],
    # )

    controller = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': webots_description,
             'use_sim_time': use_sim_time},
            diff_drive_control_params
        ],
        remappings=[
            ('/diffdrive_controller/cmd_vel_unstamped', '/cmd_vel')
        ]
    )

    # Execute/Import glados_description launch file
    glados_description_launch_file_path = get_share_file('glados_description', 'launch/description.launch.py')
    glados_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(glados_description_launch_file_path),
        launch_arguments={
            'rviz': 'False',
            'use_sim_time': use_sim_time
        }.items()
    )


    return LaunchDescription(ARGUMENTS + [
        webots,
        diffdrive_controller_spawner,
        joint_state_broadcaster_spawner,
        controller,
        # robot_state_publisher,
        glados_description,

        # Shutdown launch when Webots exits.
        RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
