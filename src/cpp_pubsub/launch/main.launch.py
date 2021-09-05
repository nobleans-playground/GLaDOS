from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_pubsub',
            namespace='talker_ns',
            executable='talker',
            name='talker_name',
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            emulate_tty=True
        ),
        Node(
            package='cpp_pubsub',
            namespace='listener_ns',
            executable='listener',
            name='listener_name',
            remappings=[
                ('/listener_ns/topic', '/talker_ns/topic'),
            ],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            emulate_tty=True
        ),
    ])



# import launch
# import launch.actions
# import launch.substitutions
# import launch_ros.actions


# def generate_launch_description():
#     return launch.LaunchDescription([
#         launch.actions.DeclareLaunchArgument(
#             'node_prefix',
#             default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
#             description='Prefix for node names'),
#         launch_ros.actions.Node(
#             package='demo_nodes_cpp', executable='talker', output='screen',
#             name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
#     ])