from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_map_transform',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'world',
                '--child-frame-id', 'map',
            ],
            output='screen',
        ),
        Node(
            package='cone_slam_sim',
            executable='cone_map_publisher',
            name='cone_map_publisher',
            output='screen',
        ),
        Node(
            package='cone_slam_sim',
            executable='cone_visualizer',
            name='cone_visualizer',
            output='screen',
        ),
    ])