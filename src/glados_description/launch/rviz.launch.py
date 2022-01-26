#!/usr/bin/env python

import os
import launch
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.actions import RegisterEventHandler, EmitEvent, DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

ARGUMENTS = [
    DeclareLaunchArgument(
        'namespace',
        default_value='glados',
        description=''
    ),
    DeclareLaunchArgument(
        'rviz',
        default_value='True',
        description=''
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

    use_sim_time = LaunchConfiguration('use_sim_time')

    glados_description_path = os.path.realpath(get_package_share_directory('glados_description'))
    urdf_path = os.path.join(glados_description_path, 'urdf')
    xacro_file = os.path.join(urdf_path, 'glados.urdf.xacro')

    rviz_file = os.path.join(glados_description_path, 'config', 'default.rviz')

    # # Execute/Import glados_description launch file
    # glados_description_launch_file_path = get_share_file('glados_description', 'launch/description.launch.py')
    # glados_description = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(glados_description_launch_file_path),
    #     launch_arguments={
    #         'rviz': 'True',
    #         'use_sim_time': use_sim_time
    #     }.items()
    # )

    rsp_params = {'robot_description': Command(['xacro ', xacro_file]), 'use_sim_time': use_sim_time}
    # 'robot_description': robot_description_xml

    # work-a-round (https://github.com/cyberbotics/webots_ros2/pull/378): don't call it robot_state_publisher
    glados_state_publisher = Node(
        name='glados_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[rsp_params],
        # condition=launch.conditions.IfCondition(publish_tf)
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[rsp_params],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(rviz_file)],
    )

    return LaunchDescription(ARGUMENTS + [
        # glados_description,
        glados_state_publisher,
        joint_state_publisher,
        rviz
    ])
