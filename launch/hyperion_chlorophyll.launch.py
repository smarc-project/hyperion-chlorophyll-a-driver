from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = '/config/config.yaml'
    return LaunchDescription([
        Node(
            package='valeport_hyperion_chlorophyll_driver',
            executable='hyperion_chlorophyll_node',
            name='hyperion_chlorophyll_node',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='valeport_hyperion_chlorophyll_driver',
            executable='chlorophyll_decoder',
            name='chlorophyll_decoder',
            output='screen',
            parameters=[config]
        ),
    ])