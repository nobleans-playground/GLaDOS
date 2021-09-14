#!/usr/bin/env python

import os
import launch
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node

from urdf2webots.importer import convert2urdf
# from webots_ros2_importer import urdf2proto

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

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

def generate_launch_description():

    base_path = os.path.realpath(get_package_share_directory('glados_description')) # also tried without realpath
    urdf_path = os.path.join(base_path, 'urdf')
    xacro_file = os.path.join(urdf_path, 'glados.urdf.xacro')
    urdf_file = os.path.join(urdf_path, 'glados.urdf')
    proto_file = os.path.join(urdf_path, 'glados.proto')
    rviz_file = os.path.join(base_path, 'config', 'default.rviz')
    assert os.path.exists(xacro_file), "glados.urdf.xacro doesn't exist in "+str(urdf_path)

    robot_description_xacro = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_xacro.toxml()

    # print(robot_description_xml)
    with open(urdf_file, 'w') as f:
        f.write(robot_description_xml)

    convert2urdf(urdf_file, proto_file, False, False, False, False, False, None, '0 1 0 0')
    # urdf2proto(--input=urdf_file)




    namespace = LaunchConfiguration('namespace')
    rviz = LaunchConfiguration('rviz')
    robot_state_publisher = LaunchConfiguration('robot_state_publisher')
    static_transform_publisher_odom = LaunchConfiguration('static_transform_publisher')
    static_transform_publisher_base_link = LaunchConfiguration('static_transform_publisher')
    joint_state_publisher = LaunchConfiguration('joint_state_publisher')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'robot_description': Command(['xacro ', xacro_file]),
    #         # 'robot_description': robot_description_xml,
    #         'use_sim_time': use_sim_time
    #     }],
    #     # condition=launch.conditions.IfCondition(publish_tf)
    # )

    static_transform_publisher_odom = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    static_transform_publisher_base_link = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    # joint_state_publisher = Node(
    #     package = 'joint_state_publisher',
    #     executable = 'joint_state_publisher'
    # )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(rviz_file)],
        condition=launch.conditions.IfCondition(rviz)
    )

    return LaunchDescription(ARGUMENTS + [
        # robot_state_publisher,
        static_transform_publisher_odom,
        static_transform_publisher_base_link,
        # joint_state_publisher,
        rviz
    ])