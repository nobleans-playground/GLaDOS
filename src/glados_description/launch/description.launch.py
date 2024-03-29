#!/usr/bin/env python

import os
import launch
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from urdf2webots.importer import convert2urdf
# from webots_ros2_importer import urdf2proto

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

import xacro

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

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

    # Hack to solve rviz not rendering cylinders
    # (https://answers.ros.org/question/374069/rviz2-only-render-boxes/)
    os.environ["LC_NUMERIC"] = "en_US.UTF-8"

    namespace = LaunchConfiguration('namespace')
    name = 'glados'
    # name = 'robleo'

    glados_description_path = os.path.realpath(get_package_share_directory('glados_description'))
    glados_simulation_path = os.path.realpath(get_package_share_directory('glados_simulation'))
    urdf_path = os.path.join(glados_description_path, 'urdf')
    xacro_file = os.path.join(urdf_path, name+'.urdf.xacro')
    assert os.path.exists(xacro_file), name+".urdf.xacro doesn't exist in "+str(urdf_path)
    urdf_file = os.path.join(urdf_path, name+'.urdf')

    proto_path = os.path.join(glados_simulation_path, 'protos')
    proto_file = os.path.join(proto_path, name+'.proto')

    # rviz_file = os.path.join(glados_description_path, 'config', 'default.rviz')

    # Convert to URDF
    robot_description_xacro = xacro.process_file(xacro_file)
    robot_description_xml = robot_description_xacro.toxml()

    # print(robot_description_xml)
    with open(urdf_file, 'w') as f:
        f.write(robot_description_xml)

    # Convert to PROTO
    convert2urdf(urdf_file, proto_file, False, False, False, False, False, None, '0 0 0 0') # rotation is axis-angle pair and not quaternion: pi rotation around z-axis (0 0 1 3.1416)
    # urdf2proto(--input=urdf_file)



    namespace = LaunchConfiguration('namespace')
    rviz = LaunchConfiguration('rviz')
    static_transform_publisher_odom = LaunchConfiguration('static_transform_publisher')
    static_transform_publisher_base_link = LaunchConfiguration('static_transform_publisher')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # Execute/Import glados_description launch file
    if launch.conditions.IfCondition(rviz):
        rviz_launch_file_path = get_share_file('glados_description', 'launch/rviz.launch.py')
        rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rviz_launch_file_path),
        )

    return LaunchDescription(ARGUMENTS + [
        static_transform_publisher_odom,
        static_transform_publisher_base_link,
        # robot_state_publisher,
        # footprint_publisher,
        rviz_launch
    ])
